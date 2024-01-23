%% %%%%%%%%%%%%%%%
% INITIALIZATION %
%%%%%%%%%%%%%%%%%%
global paperExperimentType optimizeHandPose optimizeFingers
global path_results ifMovie ifRecord
global maxExplorationSteps
global tips_normal_pub ros_on start_end_flag_pub
global explorationChance

if ifRecord
    videoName = sprintf('ExpType%s-%s.avi',paperExperimentType,objectName);
    v = VideoWriter(videoName);
    open(v);
end

%%%%%%%%%%%%%%%%%%%%%%
% I. Load hand model %
%%%%%%%%%%%%%%%%%%%%%%
hand_file = load(fullfile(path_hand_model, 'AllegroHandLeft_model.mat'));
hand = hand_file.hand;
assert(strcmp(hand.type,'AllegroHandLeft'));

if ~isfield(hand.P,'cvx_vtx_h')
    hand = obtainPalmCvxHullVertices(hand);
end

hand.q = zeros(size(hand.q)); % Set all joints to 0

global abductionIdx extensionIdx

abductionIdx = 1:4:hand.m; % indices of an-/adduction joints
extensionIdx = setdiff(1:hand.m, abductionIdx); % indices of extension/flexion joints

abductionIdx(1) = 2; % For thumb, the first abduction DoF is the 2nd DOF
extensionIdx(1) = 1; % For thumb, the first extension DoF is the 1st DOF

% Modify hand joint limits
for i = 1:length(extensionIdx)
    idx = extensionIdx(i);
    % Fix lower boundary
    if hand.limit(idx,1) < 0 % remove all negative boundaries, if <0 then 0
        hand.limit(idx,1) = 0;
    end
    hand.limit(idx,1) = hand.limit(idx,1) + deg2rad(10);
    
    % Fix upper boundary
    if hand.limit(idx,2) > deg2rad(90) % remove all boundaries exceed 90 deg, if >90 then 90
        hand.limit(idx,2) = deg2rad(90);
    end
    hand.limit(idx,2) = hand.limit(idx,2) - deg2rad(30);
end

global nFingers
nFingers = hand.n;
assert(nFingers == 4);

global latestContacts % latest contacts, used to device if need to jump out of local minimum
nLatestSteps = 5; % number of saved latest sampled contacts
latestContacts = zeros(3, nFingers, nLatestSteps);

% baseMatrices, to save bases of fingers at the initial hand pose
global baseMatrices
baseMatrices.fingers = [];
for iF = 1:nFingers
    baseMatrices.fingers = cat(3,baseMatrices.fingers,hand.F{iF}.base); % 3rd dimension is finger number
end

baseMatrices.palm = hand.P.palm_base;
baseMatrices.palm_basepoints_h = hand.P.basepoints_h; % (4,4): X (1,:), Y (2,:), Z (3,:) base points of palm, for visualization
baseMatrices.palm_cvx_vtx_h = hand.P.cvx_vtx_h; % (4,666), convex hull vertices

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% II. Optimization symbolic variables %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global params
syms xH yH zH qH1 qH2 qH3 qH4;
sym_posH = [xH, yH, zH]; % (1,3), hand pose
sym_quatH = [qH1, qH2, qH3, qH4]; % (1,4), hand quaternion, eye(3) -> [1,0,0,0]
sym_T = [symQuat2Rotm(sym_quatH), transpose(sym_posH); 0, 0, 0, 1];
hand.sym_T = sym_T;

sym_alp = sym('alp%d4',[1,4]); % (1,4), alpha of contacts
sym_phi = sym('phi%d4',[1,4]); % (1,4), phi of contacts
sym_q = reshape(transpose(sym('q%d%d',[4,4])),1,[]); % (1,16), joint angles
sym_slack = reshape(transpose(sym('delta%d%d',[4,3])),1,[]); % (1,12), slack variables: [delta11 (F1-x), delta12 (F1-y), delta13 (F1-z), delta21, ..., delta43]

X_key = [sym_posH, sym_quatH, sym_alp, sym_phi, sym_q, sym_slack];

params.sym_posH = sym_posH;
params.sym_quatH = sym_quatH;
params.sym_alp = sym_alp;
params.sym_phi = sym_phi;
params.sym_q = sym_q;
params.sym_slack = sym_slack;
params.sym_T = sym_T;
params.X_key = X_key;

% Index of symbolic variables
idx_posH = ismember(X_key, sym_posH);
idx_quatH = ismember(X_key, sym_quatH);
idx_alp = ismember(X_key, sym_alp);
idx_phi = ismember(X_key, sym_phi);
idx_q = ismember(X_key, sym_q);
idx_slack = ismember(X_key, sym_slack);

params.idx_posH = idx_posH;
params.idx_quatH = idx_quatH;
params.idx_alp = idx_alp;
params.idx_phi = idx_phi;
params.idx_q = idx_q;
params.idx_slack = idx_slack;

X0_sym = sym('X0_%d',[1,numel(X_key)]);
params.X0_sym = X0_sym;

% Optimization variable boundaries
lb_posH =-Inf*ones(size(sym_posH));
ub_posH = Inf*ones(size(sym_posH));
lb_posH(3) = 0; % Z: hand pose cannot go below table surface

if optimizeHandPose % consider as variables
    lb_quatH =-Inf*ones(size(sym_quatH));
    ub_quatH = Inf*ones(size(sym_quatH));
else
    lb_quatH = [1,0,0,0];
    ub_quatH = [1,0,0,0];
    X_key = subs(X_key, sym_quatH, [1,0,0,0]);
end

lb_alp = ones(size(sym_alp)); % [0, 0, 0, 0] % ones: force to be on fingertip
ub_alp = ones(size(sym_alp)); % [1, 1, 1, 1]

lb_phi =-pi*ones(size(sym_phi))./4;
ub_phi = pi*ones(size(sym_phi))./4;

lb_q = hand.limit(:,1).';
ub_q = hand.limit(:,2).';

lb_slack =-Inf*ones(size(sym_slack));
ub_slack = Inf*ones(size(sym_slack));

params.optimizeHandPose = optimizeHandPose;
params.optimizeFingers = optimizeFingers;
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% III. Load object point cloud data %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% To obtain object file, need to run `plyFiles2PointCloud.m` in 'database'
objectFile = load(strcat(objectName,'.mat'));

ptCloud = objectFile.pd;
objectCloud = double(objectFile.pointCloud); % (N,3)
objectNormals = double(objectFile.pointCloudNormals); % object surface normal, used in optimization formulation

objectCloud = objectCloud * 1.0*1e3; % from meter to mm
objectCloud = objectCloud * scalingFactor;

global groundTruthCloud
groundTruthCloud = objectCloud; % Save a copy of object point cloud as groundtruth in evaluation

maxWorkspace = max(objectCloud);
global maxX maxY maxZ
maxX = maxWorkspace(1);
maxY = maxWorkspace(2);
maxZ = maxWorkspace(3);

minWorkspace = min(objectCloud);
global minX minY minZ
minX = minWorkspace(1);
minY = minWorkspace(2);
minZ = minWorkspace(3);

offset = 200; % margin for ROI
plotZmin = minZ;
plotXmin = minX - offset;
plotXmax = maxX + offset;
plotYmin = minY - offset;
plotYmax = maxY + offset;
plotZmax = maxZ + offset;

global plotConfig
plotConfig.X = [plotXmin, plotXmax];
plotConfig.Y = [plotYmin, plotYmax];
plotConfig.Z = [plotZmin, plotZmax];
plotConfig.View = [120, 30]; % or: plotConfig.View = [];

global fingerColors
fingerColors = {'r','g','b','y'};

% define ROI
roiXmin = plotXmin;
roiXmax = plotXmax;
roiYmin = plotYmin;
roiYmax = plotYmax;
roiZmin = plotZmin - offset;
roiZmax = plotZmax;

global objectValues

if ros_on
    % In MuJoCo: use ROI instead of real object point cloud
    objectValues.center = [(roiXmin+roiXmax)/2, (roiYmin+roiYmax)/2, (roiZmin+roiZmax)/2];
    objectValues.dist.min = 0;
    objectValues.dist.max = max([(roiXmax-roiXmin)/2, (roiYmax-roiYmin)/2, (roiZmax-roiZmin)/2]);
else
    % In MATLAB: using object point cloud as reference for query
    objectValues.center = mean(objectCloud); % (1,3)
    objectValues.dist.min = min(dist(objectCloud, objectValues.center.')); % minimum distance from object center to object surface
    objectValues.dist.max = max(dist(objectCloud, objectValues.center.')); % maximum distance from object center to object surface
end

ub_posH = [roiXmax, roiYmax, roiZmax];
lb_posH = [roiXmin, roiYmin, roiZmin];

UB = [ub_posH, ub_quatH, ub_alp, ub_phi, ub_q, ub_slack];
LB = [lb_posH, lb_quatH, lb_alp, lb_phi, lb_q, lb_slack];

params.UB = UB;
params.LB = LB;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% IV. Constraints and objective function in symbolic form %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Load/ create sym_contacts
global sym_contacts
if isempty(sym_contacts)
    fprintf('sym_contacts is empty...\n');
    try
        sym_contacts_file = load('sym_contacts.mat');
        sym_contacts = sym_contacts_file.sym_contacts;
        fprintf('sym_contacts has been successfully loaded.\n');
    catch
        fprintf('sym_contacts cannot be loaded, create again...\n');
        sym_contacts = generateSymbolicContacts(hand,true);
        fprintf('sym_contacts has been successfully created.\n');
    end
end

%%% Load/ create symbolic functions
global path_symbolic
ifAllFilesExist = isfile(fullfile(path_symbolic,'objfun_1.m')) && isfile(fullfile(path_symbolic,'objfun.m')) && isfile(fullfile(path_symbolic,'nonl_c.m')) && isfile(fullfile(path_symbolic,'nonl_ceq.m'));
if ~ifAllFilesExist || ifGenSymExpressions
    fprintf('Generate optimization files again...\n');
    
    generateSymbolicOptimizationExpressions(hand, params); % Calculate and save symbolic constraints and objective function
    
    fprintf('Optimization files have been created.\n');
else
    fprintf('All optimization files exist.\n');
end

%%% Initialization of hand model
T = hand.T;
T(1:3,1:3) = eye(3);
T(1:3,4) = [0;0;maxZ+174.6767]; % This is the starting point of the hand pose
if objectName == "bunny"
    T(1:3,4) = [500;0;maxZ+174.6767];
end
hand.q([2,6,10,14]) = pi/2; % move fingers to vertical direction to the palm
hand_initial = updateHandTransformationForVisualization(hand, T, hand.q);

%%% Use hand and object cloud to initialize contact points

InitialContacts = zeros(nFingers,3); % Each column for one finger
for i = 1:nFingers
    symTip = hand.F{i}.symbolic.tip(1:3,4); % represented in hand reference frame, symvar: [q11,q12,q13,q14]
    numTip = double(subs(symTip, sym_q, hand.q')); % (3,1)
    numTip = numTip + T(1:3,4);
    dists = pdist2(numTip', objectCloud); % (1,N)
    [~,min_idx] = min(dists);
    InitialContacts(i,:) = objectCloud(min_idx,:);
end
InitialNormals = ones(nFingers,3); % (4,3)
InitialNormals = normalize(InitialNormals, 2, 'norm');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%batchExperiments
% V. Gaussian process model %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% parameters
theta_exp_init = [2e-1, 1.];
lengthscale_domain = [2e-1, 4e-2];
% lengthscale_domain = [2.5e-1, 8e-2];
% Input
input_data_aqp = zeros(nFingers,3);
output_data_aqp = zeros(nFingers,1);

X_fingers_initial = InitialContacts;
scale_gp = 1e-3; % 1e-2 to 1e-3
for i = 1:nFingers
   agent(i).X = X_fingers_initial(i,:);
   agent(i).theta = theta_exp_init;
   agent(i).tx = ones(1,3)/norm(ones(1,3)); % direction of local gp
   agent(i).nx = ones(1,3)/norm(ones(1,3));
   agent(i).vel = agent(i).tx; % optimized velocity
   
   agent(i).connectivity_norm = zeros(ceil(length(objectCloud)/nFingers),1);
   agent(i).gamma = zeros(ceil(length(objectCloud)/nFingers),1);
   
   input_data_aqp(i,:) = agent(i).X(1,:);
   output_data_aqp(i,:) = 0;
end

% Model instantiation
GPModel.mx = 1;
GPModel.kfnc = @squaredKern;
GPModel.theta = [8e-2, 1.];
% GPModel.theta = [1e-1, 1.];
GPModel.sigma = 3e-1;
GPModel.X = scale_gp* input_data_aqp;
GPModel.Y = output_data_aqp;
GPModel.KK = GPModel.kfnc(GPModel.X,GPModel.X,GPModel.theta);
GPModel.L = chol(GPModel.KK+GPModel.sigma^2*eye(size(GPModel.X,1)),'lower');
GPModel.alpha = GPModel.L'\(GPModel.L\(GPModel.Y -  GPModel.mx*ones(size(GPModel.Y,1),1)));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% VI. Visualization settings %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Visualize the initial configuration
% lengths of thumb links: 8.5621 + 55.4150 + 51.3996 + 59.3000 = 174.6767
% finger links: 54 + 38.4 + 43.7 = 136.10

%%% Only visualizing the initial status
fprintf('\Visualizing initial configuration...\n');
h = figure;
hold on;
h.WindowState = 'maximized';
h.NextPlot = 'replacechildren';

mySGplotHand(hand_initial);
scatter3(objectCloud(:,1),objectCloud(:,2),objectCloud(:,3), 10*scalingFactor, 'MarkerFaceColor', greyColor, 'MarkerFaceAlpha', 0.2, 'MarkerEdgeColor', 'none', 'LineWidth', 0.5);
scatter3(InitialContacts(:,1),InitialContacts(:,2),InitialContacts(:,3), 50*scalingFactor, 'r', 'filled');
view([120,30]);
xlabel('X');
ylabel('Y');
zlabel('Z');
grid off;
title('Initial configuration');
hold off;
saveas(gcf, fullfile(path_results, sprintf('ExpType%s-%s-initial_configuration.jpg',paperExperimentType,objectName)));

movieFrames = []; % To save movie frames
sampledCloud = []; % (N,3) Save sampled data points
latestNSamples = []; % (bufferSize,3) save the latest N=100 samples to selectively exclude neighbour points when updating GPR model
bufferSize = 100; % max. number of samples to keep in the buffer "latestNSamples"
removeSize = 10; % remove max. 10 from the points sampled from latest movement

%%% Visualizing the movement of hand from 'hand' (loaded model) to 'hand_initial' (starting point)
if ifMovie
    M = handTrajectoryVisualization(hand, hand_initial, timeSteps, objectCloud);
    movieFrames = cat(1, movieFrames, M);
    
    % if ifRecord
    %     for i = 1:length(M)
    %         writeVideo(v,M(i));
    %     end
    % end
end
hand = hand_initial; % hand_initial is useful only if movie the initialization step

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% VII. Establish first contact %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('\nEstablish first contact...\n');
oldHand = hand;
this_contacts.positions = transpose(InitialContacts);
this_contacts.normals = transpose(InitialNormals); % pointing to Z+ direction
next_contacts = this_contacts;

stepCounter = 1; % Initial step
vecT = zeros(3,4);
hand.contacts = this_contacts;
[hand,initial_exitflag] = solveFingerAndWristIK(hand, sym_contacts, this_contacts, next_contacts, vecT);

if initial_exitflag < 0
    x_new = [];
    error('Initial contacts cannot be reached.');
else
    x_new = transpose(hand.contacts.positions); % (4,3) % just sampled points from each fingertip
    sampledCloud = cat(1, sampledCloud, x_new); % (N,3)

    latestNSamples = sampledCloud(max(1,end-bufferSize+1):end, :); % latest "bufferSize" samples

    y_new = zeros(size(x_new,1),1); % (4,1)
    GPModel = update_gp(GPModel, scale_gp*x_new, y_new);

    % objectCloud = removeFromCloud(objectCloud, x_new);

    if ifMovie
        M = handTrajectoryVisualization(oldHand, hand, timeSteps, objectCloud, sampledCloud); % M is movie frame
        movieFrames = cat(1,movieFrames,M);
        saveas(gcf, fullfile(path_results, sprintf('ExpType%s-%s-first_contact.jpg', paperExperimentType, objectName)));
        
        % if ifRecord
        %     for i = 1:length(M)
        %         writeVideo(v,M(i));
        %     end
        % end
    end
end

if_perturbation = false(1,nFingers);

%% %%%%%%%%%%%%%%%%%%%%%%
% MAIN EXPLORATION LOOP %
%%%%%%%%%%%%%%%%%%%%%%%%%
% x_new = []; % New contact points to register
% counter = 0; % Counter to extract points from pointCloud
% rng('default')

last_vecT = ones(4,3);
vecT = rand(4,3) - 0.5;
vecT = normalize(vecT, 2, 'norm');
for iF = 1:nFingers
    vecT(iF,:) = vecT(iF,:) - vecT(iF,:) * hand.contacts.normals(:,iF) * hand.contacts.normals(:,iF)';
end
% vecT = [[0,1,0];[0,1,0];[0,1,0];[0,1,0]];


% vecT_mode = 'fully_random';  % use fully random vecT, within a cone of the last vecT
% vecT_mode = 'GP_projection'; % use the vecT from GP, within a cone of the last vecT
 vecT_mode = 'GP_projection'; % use the vecT from GP, within a cone of the last real direction
% vecT_mode = 'original';   % use the vecT from GP, no projection

% theta_cone = pi / 6 * 1.0;
theta_cone = pi / 6;
% theta_cone = pi / 6 * 1.0  * 0.5;

perturbation_mode = false;
% while length(sampledCloud) < length(objectCloud) % Stop criterion

%%%% send a message to start MuJoCo running
if ros_on
    ros_msg = rosmessage(start_end_flag_pub);
    ros_msg.Data = [1,0]';
    send(start_end_flag_pub, ros_msg);
end

while stepCounter <= maxExplorationSteps
    last_vecT = vecT;
    if isfield(hand.contacts, 'moving_direction') 
        last_vecT = hand.contacts.moving_direction'; % (3,4)
    end
    fprintf('\n %s-%s : Trial %d - Exploration step: %03d\n', batchExperiments{1,1}, objectName, trial, stepCounter);
    stepCounter = stepCounter + 1;
    oldHand = hand;
    this_contacts = oldHand.contacts;
    
    %% GPR calculates the next exploration tangential direction
    agent = update_agents(GPModel, agent, x_new, lengthscale_domain, hand.contacts.normals); % Update GP
    [vecN, vecT] = get_fingers_nt(agent); % Get normal and tangent vectors (4,3) in local frame
    
    vecT = normalize(vecT, 2, 'norm');
    vecN = normalize(vecN, 2, 'norm');
    
    switch vecT_mode
        case 'fully_random'
            fprintf('fully random vecT.\n');
            vecT = rand(4,3) - 0.5;
            vecT = normalize(vecT, 2, 'norm'); 
            last_vecT = normalize(last_vecT, 2, 'norm'); 
            
            for iF = 1:nFingers
                cos_alpha = vecT(iF,:) * last_vecT(iF,:)';
                if cos_alpha < cos(theta_cone)  % when the angle is bigger than theta_cone
                    e_axis = cross(last_vecT(iF,:), vecT(iF,:));
                    e_axis = normalize(e_axis, 'norm');

                    % rotate last_vecT by the angle-axis representation
                    vecT(iF,:) = cos(theta_cone) * last_vecT(iF,:) + ...
                        sin(theta_cone) * cross(e_axis, last_vecT(iF,:)) + ...
                        (1 - cos(theta_cone)) * (e_axis * last_vecT(iF,:)') *e_axis;
                end

                % project to the tangential plane of normals
%                 vecT(iF,:) = vecT(iF,:) - vecT(iF,:) * hand.contacts.normals(:,iF) * hand.contacts.normals(:,iF)';
                vecT(iF,:) = normalize(vecT(iF,:), 'norm');
                if perturbation_mode
                    if if_perturbation(iF)
                        vecT(iF,:) = vecT(iF,:) .* rand(1,3) * 0.3;
                        vecT(iF,:) = normalize(vecT(iF,:), 'norm') * 1.5;
                    end
                end
            end
        %     vecT = normalize(vecT, 2, 'norm');
        
        case 'GP_projection'
            for iF = 1:nFingers
                cos_alpha = vecT(iF,:) * last_vecT(iF,:)';
                if cos_alpha < cos(theta_cone)  % when the angle is bigger than theta_cone
                    e_axis = cross(last_vecT(iF,:), vecT(iF,:));
                    e_axis = normalize(e_axis, 'norm');

                    % rotate last_vecT by the angle-axis representation
                    vecT(iF,:) = cos(theta_cone) * last_vecT(iF,:) + ...
                        sin(theta_cone) * cross(e_axis, last_vecT(iF,:)) + ...
                        (1 - cos(theta_cone)) * (e_axis * last_vecT(iF,:)') *e_axis;
                end

%                 project to the tangential plane of normals
%                 vecT(iF,:) = vecT(iF,:) - vecT(iF,:) * hand.contacts.normals(:,iF) * hand.contacts.normals(:,iF)';
                vecT(iF,:) = normalize(vecT(iF,:), 'norm');
                if if_perturbation(iF)
                    vecT(iF,:) = vecT(iF,:) * 3;
                end
            end
       
%         case 'GP_projection_2'
        case 'original'
            if if_perturbation(iF)
                    vecT(iF,:) = vecT(iF,:) * 1.5;
            end
        otherwise
            error('Unknown mode for vecT.');
               
       
        
        
    end
    
    
    
    
    
%     if rand <= explorationChance % Chances to take a random exploration on the object surface
%         vecT = rand(4,3);
%         vecT = normalize(vecT, 2, 'norm') *4;  % try to fully random
%         fprintf('Let`s take a random walk...\n');
    
% %     if 0
% %         tmp = 1;
%     else
%      %%% If intend to add noise to the tangential vector
%         for iF = 1:nFingers
%             if sum(if_perturbation(2:4))
%                 fprintf('Add noise to disturb tangential direction of finger %d...\n', iF); % (4,3)
% 
%                 p_verT = vecT(iF,:);
%                 p_verT = p_verT.*rand(size(p_verT)) + 1e-4;
%                 p_verT = normalize(p_verT, 'norm');
%                 vecT(iF,:) = p_verT * 5;
% 
% %                 vecT(iF,:) = vecT(iF,:) * 3;
%             end
%         end
%    end
    

        
    
    if any(isnan(vecT))
        vecT = zeros(size(vecT));
    end
    if any(isnan(vecN))
        vecN = zeros(size(vecN));
    end
    
    step_exitflag = 0; % flag indicating if a step-planning is successful (>0) or not (<=0)
    failure_counter = 1;
    old_next_contact = this_contacts; % save the last desired exploration point
    
    % Visualize tangential direction given by GPR model
    for iF = 1:nFingers
        origin = this_contacts.positions(:,iF); % (3,1)
        tangential = transpose(vecT(iF,:)); % (3,1)
        vec_end = origin + tangential*100;
        plot3([origin(1) vec_end(1)],[origin(2) vec_end(2)],[origin(3) vec_end(3)],strcat(fingerColors{iF},'-'),'LineWidth',5);
    end
    
    while ~(step_exitflag > 0)
        % Generate the next desired exploration positions
        % Notice that this function should be replaced in robotic experiment:
        % "objectCloud should remain unknown during exploration; and next
        % contacts should be generated as a "virtual" contacts; and the real
        % contacts are detected by the sensor.
        if_find_next_contacts = false;

        while ~if_find_next_contacts
            
            next_contacts = generateNextStepSamples(hand, this_contacts, objectCloud, vecT, vecN); % (3,4)

            next_positions = next_contacts.positions;
            
            if_find_next_contacts = norm(old_next_contact.positions - next_positions) > 10; % detect if 'next_contacts' remains the same in two consecutive loops
            
            if if_find_next_contacts
                % Visualize the exploration targets as unfilled circles
                scatter3(next_positions(1,1),next_positions(2,1),next_positions(3,1),100*scalingFactor,'MarkerEdgeColor','r','Marker','o','LineWidth',3);
                scatter3(next_positions(1,2),next_positions(2,2),next_positions(3,2),100*scalingFactor,'MarkerEdgeColor','g','Marker','o','LineWidth',3);
                scatter3(next_positions(1,3),next_positions(2,3),next_positions(3,3),100*scalingFactor,'MarkerEdgeColor','b','Marker','o','LineWidth',3);
                scatter3(next_positions(1,4),next_positions(2,4),next_positions(3,4),100*scalingFactor,'MarkerEdgeColor','y','Marker','o','LineWidth',3);
                
                % scatter3(next_positions(1,:),next_positions(2,:),next_positions(3,:),50*scalingFactor,'c','filled');
                                
                % Visualize surface normal directions on exploration targets
                %{
                for iF = 1:nFingers
                    origin = next_positions(:,iF);
                    normal = next_contacts.normals(:,iF);
                    normal_end = origin + normal*100;
                    plot3([origin(1) normal_end(1)],[origin(2) normal_end(2)],[origin(3) normal_end(3)],strcat(fingerColors{iF},'-'),'LineWidth',2.5);
                end
                %}
                
            else
                % vecT = vecT.*rand(size(vecT));
                % vecT = normalize(vecT, 2, 'norm');

                for iF = 1:nFingers % remove next contacts from objectCloud to prevent getting into the same infeasible points again
                    tmp_p = next_positions(:,iF); % (3,1)
                    objectCloud(ismember(objectCloud,tmp_p','rows'),:) = []; % (N,3)
                end
            end
        end

        %% %%%%%%%%%%%%%%%%%%
        % Main Optimization %
        %%%%%%%%%%%%%%%%%%%%%
        switch experimentType

            case 'full'
                [hand, step_exitflag, if_perturbation] = solveFingerAndWristIK(hand, sym_contacts, this_contacts, next_contacts, vecT);

            case 'gpr' % only test the gpr model for exploration algorithm
                hand.contacts.positions = this_contacts.positions;
                hand.contacts.normals = this_contacts.normals;
                step_exitflag = 1;
                if_perturbation = false(1,nFingers);

            case 'mixed'
                if mod(stepCounter,saveFrameStep)==0 % the frame to save hand configuration
                    [hand, step_exitflag, if_perturbation] = solveFingerAndWristIK(hand, sym_contacts, this_contacts, next_contacts, vecT);
                else
                    hand.contacts.positions = this_contacts.positions;
                    hand.contacts.normals = this_contacts.normals;
                    step_exitflag = 1;
                    if_perturbation = false(1,nFingers);
                end
        end

        if step_exitflag <= 0
            % if the current "next_contacts" cannot be reached, change the next_contacts by modifying vecT
            fprintf('No solution, adjust vecT (failure counter: %d)...\n', failure_counter);
            failure_counter = failure_counter + 1; % increase the number in each failed trial, to get out of local failures faster
            vecT = vecT.*rand(size(vecT)) + 1e-4;
            vecT = normalize(vecT, 2, 'norm') * exp(failure_counter);
            if failure_counter == 40
                error('Abort: failure continued for %d times.\n', failure_counter);
            end
        else
            failure_counter = 1; % success, reset counter
        end
        
    end
        
    %%% Update GP model
    if isfield(hand, 'contacts_one_step') && ~isempty(hand.contacts_one_step)
        x_new = transpose(hand.contacts_one_step);
        fprintf('x_new number of points %d.\n', size(x_new, 1)); % (n,3)
    else
        x_new = transpose(hand.contacts.positions); % (n,3)
    end

    %%% downsample x_new by removing M points that are closest to the buffer
    if size(latestNSamples, 1) >= bufferSize % enough sampled points, filter out x_new
        D = pdist2(latestNSamples, x_new, 'euclidean'); % (bufferSize,n), vector distance btw (bufferSize,3) and (n,3)
        minD = min(D, [], 1); % (1,n)
        [~, idxList] = sort(minD, 'ascend'); % sort min. distance, from min to max
        removeList = idxList(1:min(end,removeSize)); % idx of samples to remove, they have min. distance to samples in the buffer
        
        if ~isempty(removeList)
            x_new(removeList,:) = []; % remove these points from x_new
        end
    end
    
    % if isfield(hand, 'current_contact_label')
    %     y_new = hand.current_contact_label; % (4,1)
    % else
    %     y_new = zeros(size(x_new,1),1);
    % end
    
    y_new = zeros(size(x_new,1), 1);
    if isfield(hand, 'current_fingertips')  % add fingertip positions as non-contact points
        x_new_1 = transpose(hand.current_fingertips);  % (4,3)
        y_new_1 = ones(size(x_new_1, 1), 1);
        x_new = [x_new; x_new_1];
        y_new = [y_new; y_new_1];
    end
    
    GPModel = update_gp(GPModel, scale_gp*x_new, y_new);
    
    sampledCloud = cat(1, sampledCloud, transpose(hand.contacts.positions)); % (4*step, 3)

    latestNSamples = sampledCloud(max(1,end-bufferSize+1):end, :); % update buffer
    
    if ifMovie
        iM = handTrajectoryVisualization(oldHand, hand, timeSteps, objectCloud, sampledCloud, vecN, vecT, this_contacts, next_contacts);
        movieFrames = cat(1, movieFrames, iM);

        if mod(stepCounter, saveFrameStep)==0
            saveas(gcf, fullfile(path_results, sprintf('ExpType%s-%s-F%03d.jpg', paperExperimentType, objectName, stepCounter)));
        end
        
        % if ifRecord
        %     for i = 1:length(M)
        %         writeVideo(v,M(i));
        %     end
        % end
    end
    
    % send normal direction to ROS
    if ros_on
        nx_tips = zeros(12,1);
        for i = 1:4
            nx_tmp = get_gp_normal(GPModel, scale_gp*hand.ftips(:,i)'); % out of the surface
            nx_tips(3*(i-1)+1 : 3*i) = nx_tmp / norm(nx_tmp);
        end
        
        ros_msg = rosmessage(tips_normal_pub);
        ros_msg.Data = nx_tips;
        send(tips_normal_pub, ros_msg);
    end

    
end

% The final step
if ifMovie
    save(sprintf('Movie-%s.mat',objectName),'movieFrames');
    FPS = 2;
    movie(movieFrames, 1, FPS); % Visualize the exploration sequence as a movie    
    saveas(gcf, fullfile(path_results, sprintf('ExpType%s-%s-final_configuration.jpg',paperExperimentType,objectName)));
    % close(v);
end






%% %%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gaussian Process Functions %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Nx,Nt] = get_fingers_nt(agent)
    nof = length(agent);
    Nx = zeros(nof,3);
    Nt = zeros(nof,3);
    for j = 1:nof
        Nx(j,:) = agent(j).nx;
        Nt(j,:) = agent(j).tx;
    end
end

function finger_out = update_agents(GPModel,agent,xnew,lengthscale_domain, normals)
    finger_out = agent;
    len0 = lengthscale_domain(1);
    lenf = lengthscale_domain(2);
    no_fingers = length(finger_out);
    counter = size(finger_out(1).X,1);

    if(size(xnew,1)==no_fingers)
       for j=1:no_fingers
            finger_out(j).X = [finger_out(j).X;xnew(j,:)];
            KKs_aqp = GPModel.kfnc(GPModel.X,finger_out(j).X(end,:),GPModel.theta);
            kn_ = mean(KKs_aqp);
            finger_out(j).connectivity_norm(counter+1) = .2*kn_ + 0.8*finger_out(j).connectivity_norm(counter);
       end
    end
    
    for j = 1:no_fingers
        x = finger_out(j).X(end,:);
        y = 0;        
        idx = knnsearch(GPModel.X,x,'K',20);
        std_knn = norm(std(GPModel.X(idx,:)));
        p = (std_knn/0.1)^6;
        gamma_des = max(0,min(1, p ));
        if (counter > 1)
        gam_c_dot = poslin(finger_out(j).connectivity_norm(counter) - finger_out(j).connectivity_norm(counter-1));
        else
            gam_c_dot = 0;
        end
        beta = 10.;
        gamma_delta = -beta * gam_c_dot*(finger_out(j).gamma(counter) - gamma_des);
        finger_out(j).gamma(counter+1)= max(0,min(1, gamma_delta + finger_out(j).gamma(counter)));

        lensc = (1-finger_out(j).gamma(counter+1))*len0 + finger_out(j).gamma(counter+1)*lenf;
        finger_out(j).theta = [lensc, 1.];

        Kpoint = min(100,size(GPModel.X,1));
        idxx = knnsearch(GPModel.X,x,'K',Kpoint);
        X_temp = GPModel.X(idxx,:);
        Y_temp = GPModel.Y(idxx,:);
        K_aqp = GPModel.kfnc(X_temp,X_temp,finger_out(j).theta);
        L_aqp = chol(K_aqp+GPModel.sigma^2*eye(size(X_temp,1)),'lower');
        alp_aqp = L_aqp'\(L_aqp\(Y_temp - GPModel.mx*ones(size(Y_temp,1),1) )); 

        Ks = GPModel.kfnc(X_temp,x,finger_out(j).theta);
        dKx = (-1/finger_out(j).theta(1)^2)*(repmat(Ks,1,3)).*(x-X_temp);
        nxx = alp_aqp'*dKx;
%         nxx = what comes from MuJuco
        nxx = normals(:,j)';
        if(norm(nxx) < 1e-6)
            finger_out(j).nx  = ones(1,3);
        else
            finger_out(j).nx  = nxx/norm(nxx);
        end

        beta = L_aqp'\(L_aqp\dKx);
        vx =-2*Ks'*beta;
        txm = vx - (vx*finger_out(j).nx')*finger_out(j).nx ;

        if(norm(txm) < 1e-6)
            if(size(finger_out(j).X,1)<2)
                txm = ones(1,3)/norm(ones(1,3));
            else
%                 txm = ones(1,3)/norm(ones(1,3));
%                 t = finger_out(j).X(end,:)-finger_out(j).X(end-1,:);
                   t = rand(1,3)+rand(1,3);
                   t = t/norm(t);
                   txm = t - (t*finger_out(j).nx')*finger_out(j).nx ;
                txm = txm/norm(txm);
            end
        else
            txm = txm/norm(txm);
        end
        finger_out(j).tx  = txm;
    end
end

function gp = update_gp(GPModel,x_new,y_new)
    gp = GPModel;
    n = size(gp.X,1);
    gp.Y = [gp.Y;y_new];
    gp.X = [gp.X;x_new];
    gp.KK = gp.kfnc(gp.X,gp.X,gp.theta);
    gp.L = chol(gp.KK+gp.sigma^2*eye(size(gp.X,1)),'lower');
    gp.alpha = gp.L'\(gp.L\(gp.Y - gp.mx*ones(size(gp.Y,1),1)));
end

function y = get_gp_prediction(gpmodel,x)
    Kx = gpmodel.kfnc(gpmodel.X,x,gpmodel.theta);
    y = gpmodel.mx*ones(size(Kx,2),1) +  Kx'*gpmodel.alpha;
end


function nx = get_gp_normal(gpmodel,x)
    alp_aqp = gpmodel.alpha; 
    Ks = gpmodel.kfnc(gpmodel.X,x,gpmodel.theta);
    dKx = (-1/gpmodel.theta(1)^2)*(repmat(Ks,1,3)).*(x-gpmodel.X);
    nx = alp_aqp'*dKx;     
end

function knm = squaredKern(XN,XM,theta)
    knm = (exp(theta(2))^2)*exp(-(pdist2(double(XN),double(XM)).^2)/(2*theta(1)^2)); 
end