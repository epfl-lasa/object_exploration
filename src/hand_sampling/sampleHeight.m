function batchSamples = sampleHeight(ptCloud, explore_height, config)

precision = config.precision;
if_plot = config.if_plot;
rot_mu = config.rot_mu; % average rotation angles
rot_std = config.rot_std; % rotation standard deviation
num_trials = config.num_trials; % number of rotation steps
num_fingers = config.num_fingers; % number of fingers
ws_margin = config.ws_margin; % workspace margin
noise_sigma = config.noise_sigma; % noise at sampling point coordination, represented as the percentage of motion range

step_length = config.step_length; % finger motion step distance (Cartesian distance)

%% Define Workspace
x_span = abs(ptCloud.XLimits(2)-ptCloud.XLimits(1)); % define range of workspace
y_span = abs(ptCloud.YLimits(2)-ptCloud.YLimits(1));
% z_span = abs(ptCloud.ZLimits(2)-ptCloud.ZLimits(1));

Xmin = ptCloud.XLimits(1) - ws_margin*x_span;
Xmax = ptCloud.XLimits(2) + ws_margin*x_span;
Xmid = (Xmax+Xmin)/2;

Ymin = ptCloud.YLimits(1) - ws_margin*y_span;
Ymax = ptCloud.YLimits(2) + ws_margin*y_span;
Ymid = (Ymax+Ymin)/2;

% Zmin = ptCloud.ZLimits(1) - ws_margin*z_span;
% Zmax = ptCloud.ZLimits(2) + ws_margin*z_span;
% Zmid = (Zmax+Zmin)/2;

%% Define Fingers
% starting positions of fingertips
trajStr = [Xmin, Ymid, explore_height;...
    Xmid, Ymin, explore_height;...
    Xmax, Ymid, explore_height;...
    Xmid, Ymax, explore_height];

% ending positions of fingertips
trajEnd = [Xmax, Ymid, explore_height;...
    Xmid, Ymax, explore_height;...
    Xmin, Ymid, explore_height;...
    Xmid, Ymin, explore_height];

%% Exploration Settings
% extract the current groundtruth of samples, (:,3)
idx = abs(ptCloud.Location(:,3)-explore_height) < precision;
ObjectSurface = ptCloud.Location(idx,:);

if if_plot
    figure, hold on;
    scatter3(ObjectSurface(:,1),ObjectSurface(:,2),ObjectSurface(:,3));
    title('Groundtruth Samples');
    axis equal;
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    hold off;
end

batchSamples = []; % (:,3), Save sampled data

%%% for visualization
if if_plot
    figure, hold on;
    cmap = jet(num_trials);
    axis equal;
    grid on;
    title('Visualization of Sampling at Given Height');
end

angle_cumulative = 0; % cumulative rotation angle

actv_idx = [true,false,true,false]; % indicate which fingers are active (rotating)

for i = 1:num_trials % num of exploration trials
    %%% Step 1: Sample (in contact, register points)
    tmp_samples = []; % zeros(num_fingers,3); % save sampled points from this exploration step
    
    for j = 1:num_fingers
        if ~actv_idx(j) % this finger is fixed to hold the object
            continue;
        end
        
        p_str = trajStr(j,:); % (1,3)
        p_end = trajEnd(j,:); % (1,3)
        
        traj_direction = (p_end-p_str)/norm(p_end-p_str);
        p_now = p_str;
        
        in_contact = false;
        
        while (~in_contact) && (norm(p_now-p_end)>precision)
            
            % the fingetip moves along the trajectory
            p_now = p_now + step_length*traj_direction;
            
            % calculate the distance between the current sample point to
            % the real object
            dist2obj = vecnorm(ObjectSurface-p_now, 2, 2);
            [min_dist, min_idx] = min(dist2obj);
            
            if min_dist < 2*precision
                in_contact = true;
                contact_point = ObjectSurface(min_idx,:);
                contact_point = normrnd(contact_point, noise_sigma); % add noise to the sampling
                tmp_samples = cat(1,tmp_samples,contact_point); % (num_fingers,3)
            end
        end
        
        if ~in_contact
            disp('No contact detected. Error.');
        end
    end
    
    %%% Step 2: Visualization of sampling results
    tmp_samples = transpose(rotz(-angle_cumulative) * tmp_samples'); % (num_fingers,3)
    
    % plot step results
    if if_plot
        scatter3(tmp_samples(:,1),tmp_samples(:,2),tmp_samples(:,3),...
            'MarkerFaceColor',cmap(i,:));
        hold on;
    end
    
    %%% Step 3: Rotate object (w.r.t. hand reference frame)
    batchSamples = cat(1,batchSamples,tmp_samples); % (:,3), register sampled data

    angle_rot = normrnd(rot_mu,rot_std); % generate rotation angle for next rotation
    ObjectSurface = transpose(rotz(angle_rot) * ObjectSurface'); % (,3), 'rotate the object'
    
    %%% Step 4: Estimate the angle of rotation
    %{
        Estimate rotate angle of object:

        Fingers are divided into two groups and move in alternative: one group
        of fingers are in contact with objects, while the other grooup fingers
        rotate to the new position.

        - robot command (rotating fingers)
        - sensor (stabilizing fingers)

        - (additional) similarity (data similarity ?)
    %}
    
    %%% Use the estimation of the angle: a linear combination of all
    %%% signals: (1) robot command, (2) estimation based on sensor signal,
    %%% (3) similarities of estimated signals and collected dataset (TBD)
    
    eta = 0.8; % weight to balance the estimations
    
    % (1) robot is given the rotation command, mean is angle_rot
    estimate_cmd = normrnd(angle_rot, noise_sigma);
    
    % (2) estimation of rotation angle based on signal similarity
    estimate_signal = normrnd(angle_rot, noise_sigma); % [TODO]
    
    % (3) correction term, based on the sensor signal estimation
    delta = 0; % [TODO]
    
    angle_estimate = eta*estimate_cmd + (1-eta)*estimate_signal + delta;
    angle_cumulative = angle_cumulative + angle_estimate; % in the end, add the estimated angle to the accumulative angle

    %%% Step 5: switch the active fingers and inactive fingers
    actv_idx = ~actv_idx;
end

if if_plot
    hold off;
end

disp('Sampling at given height completed.');