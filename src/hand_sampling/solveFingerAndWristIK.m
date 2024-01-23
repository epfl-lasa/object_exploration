% This function solves the IK of entire hand, including wrist (represented
% by hand reference frame) while considering the IK of fingers at the same
% time.
% 
% Input: desired contact points and surface normals for each fingertip
% - vecT: (3,4), tangential direction on the object surface from the current
% point, model uncertainty decreases fastest along this direction. This is
% the desired exploration direction.
% - vecN: (3,4), normal direction of the explored surface. Velocity should be 0 in
% this direction (perpendicular).
% - Hz: scalar, the sampling frequency.
% 
% Output: hand configuration at next exploration time step    
    
function [hand, exitflag, if_perturbation] = solveFingerAndWristIK(hand, sym_contacts, this_contacts, next_contacts, VecT)
    % inputArgSet = cell(1,nargin);
    % for i = 1:nargin
    %     inputArgSet{end+1} = inputname(i);
    % end

    global params nFingers
    global baseMatrices
    global objectValues
    global t_fmincon % For time evaluation
    global ros_on
    % global opts
    global pose_joints_pub contacts_sub if_contacts_feedback  % for ROS publisher
    global cost_fun
    
    global groundTruthCloud
    global latestContacts
    
    if_perturbation = false(1,nFingers); % if need to add noise to disturb vecT for planning next targets
    
    % Fomulation of optimization problem
    X_key = params.X_key;

    % Optimization boundary
    LB = params.LB;
    UB = params.UB;
    
    % Linear constraints
    Aeq = [];
    beq = [];
    A = [];
    b = [];
    
    %% Formulation of optimization problem
    %% Initial values
    init_posH = hand.T(1:3,4).'; % (1,3), Initial value
    init_quatH = rotm2quat(hand.T(1:3,1:3)); % (1,4), Initial value, correspond to R = eye(3)
    init_q = hand.q.'; % (1,16), q11, q12, q13, q14, q21, ..., q44
    init_slack = zeros(size(params.sym_slack)); % (1,12)
    init_alp = zeros(1,nFingers); % (1,4)
    init_phi = zeros(1,nFingers); % (1,4)
    for iF = 1:nFingers % Assign initial values of alpha and phi using previous contact values
        init_alp(iF) = hand.F{iF}.Link{end-1}.contact.crdCyl.alp;
        init_phi(iF) = hand.F{iF}.Link{end-1}.contact.crdCyl.phi;
    end
    X_init = [init_posH, init_quatH, init_alp, init_phi, init_q, init_slack]; % (1,43)
    
    % Symbolic Expression of Nonl. Constraints and Objective Functions
    nonlConstraints = @(X)optExplore_nonlcon(X, next_contacts, baseMatrices, objectValues, init_quatH); % Nonlinear Constraints interface
    objFunction = @(X)optExplore_objfun(X, X_init, VecT); % Objective function interface

    % Configuration of optimization problem
    % global opts

    % Set up shared variables with outfun (Output function)
    history.x = [];
    history.fval = [];
    searchdir = [];

    opts = optimoptions(@fmincon,...
        'OutputFcn',@outfun,... 
        'Algorithm','sqp',...% 'active-set', 'interior-point', 'sqp', 'trust-region-reflective', or 'sqp-legacy'
        'SpecifyObjectiveGradient',false,...
        'SpecifyConstraintGradient',false,...
        'Display','off',...% 'final', 'off', 'notify', 'notify-detailed', 'final', 'final-detailed', 'iter', or 'iter-detailed'
        'OptimalityTolerance',1e-2,...
        'FunctionTolerance',1e-2,...
        'MaxFunctionEvaluations',1e4,...
        'MaxIterations',1e4,...
        'StepTolerance',1e-2,...
        'CheckGradients',false,...
        'FiniteDifferenceType','central',...
        'FiniteDifferenceStepSize',1e-2,...
        'ConstraintTolerance',1e-2);

    function stop = outfun(x,optimValues,state)
        stop = false;
        switch state
            case 'init'
                % hold on
            case 'iter'
                % Concatenate current point and objective function value with history
                history.fval = [history.fval; optimValues.fval];
                history.x = [history.x; x(:)']; % x must be a row vector
                
                % Concatenate current search direction with searchdir
                searchdir = [searchdir;... 
                optimValues.searchdirection'];
            case 'done'
                % hold off
            otherwise
        end
    end
    
    fprintf('Solving fmincon...');
    tic;
    [X_sol, fval, exitflag, output] = fmincon(objFunction, X_init, A, b, Aeq, beq, LB, UB, nonlConstraints, opts);
    t = toc;
    fprintf('%d seconds.\n', t);

    t_fmincon(end+1) = t; % 1.13 +/- 0.73
    
    if ~isfield(cost_fun,'nSteps')
        cost_fun.('nSteps') = 1;
    else
        cost_fun.('nSteps') = cost_fun.('nSteps') + 1;
    end
    
    % Calculate each term in objective function for later evaluation
    if exist('objterm_joints','file')
        if ~isfield(cost_fun,'joints')
            cost_fun.('joints') = struct('val',[],'t',[]); % val: value; t: time
        end
        tic;
        val = objterm_joints(X_sol);
        t = toc;
        cost_fun.('joints').('val')(end+1) = val;
        cost_fun.('joints').('t')(end+1) = t;
    end
    
    if exist('objterm_slack','file')
        if ~isfield(cost_fun,'slack')
            cost_fun.('slack') = struct('val',[],'t',[]);
        end
        tic;
        val = objterm_slack(X_sol);
        t = toc;
        cost_fun.('slack').('val')(end+1) = val;
        cost_fun.('slack').('t')(end+1) = t;
    end
    
    if exist('objterm_isotropy','file')
        if ~isfield(cost_fun,'isotropy')
            cost_fun.('isotropy') = struct('val',[],'t',[]);
        end
        tic;
        val = objterm_isotropy(X_sol);
        t = toc;
        cost_fun.('isotropy').('val')(end+1) = val;
        cost_fun.('isotropy').('t')(end+1) = t;
    end
    
    if exist('objterm_omega','file')
        if ~isfield(cost_fun,'omega')
            cost_fun.('omega') = struct('val',[],'t',[]);
        end
        tic;
        val = objterm_omega(X_sol);
        t = toc;
        cost_fun.('omega').('val')(end+1) = val;
        cost_fun.('omega').('t')(end+1) = t;
    end
    
    if exist('objterm_variance','file')
        if ~isfield(cost_fun,'variance')
            cost_fun.('variance') = struct('val',[],'t',[]);
        end
        tic;
        val = objterm_variance(X_sol,X_init,VecT);
        t = toc;
        cost_fun.('variance').('val')(end+1) = val;
        cost_fun.('variance').('t')(end+1) = t;
    end
    
    if exitflag > 0
        sol_q = X_sol(params.idx_q);
        sol_handHT = double(subs(params.sym_T, X_key(:), X_sol(:)));
        sol_alp = X_sol(params.idx_alp);
        sol_phi = X_sol(params.idx_phi);
        
        sol_positions = double(subs(sym_contacts.positions, X_key(:), X_sol(:))); % (3,4)
        sol_normals = double(subs(sym_contacts.normals, X_key(:), X_sol(:))); % (3,4)
        
        % sol_slack = X_sol(params.idx_slack);
        % sol_slack = reshape(sol_slack,3,4);
        % err_slack = vecnorm(sol_slack,2,1); % 2-norm, along the 1st dimension
        % fprintf('Norms of slack variables: %d, %d, %d, %d\n', err_slack(1), err_slack(2), err_slack(3), err_slack(4));
        
        % err_target = vecnorm(sol_positions - next_contacts.positions, 2, 1); % distance btw obtained positions and desired positions
        % fprintf('Aberration from desired contacts: %d, %d, %d, %d\n', err_target(1), err_target(2), err_target(3), err_target(4));
        
        dist_to_groundtruth = zeros(1,nFingers);
        for i = 1:nFingers
            fingerTip = sol_positions(:,i); % (3,1)
            dists = pdist2(fingerTip', groundTruthCloud); % (N,3), must have same number of columns
            dist_to_groundtruth(i) = min(dists);
        end
        
        if max(dist_to_groundtruth) <= 100 % exitflag > 0 % max error range: 20 mm [based on exploration results]
            %% %%%%%%%%%%%%%%%%
            % Successful Case %
            %%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Step 1: Send desired hand pose (sol_handHT) and finger joint
            % angles (sol_q) to MuJoCo via ROS
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if ros_on
                ros_msg = rosmessage(pose_joints_pub);
                T = sol_handHT;
                T(1:3,4) = T(1:3,4) / 1000.; % mm to m
                ros_msg.Data = [T(:); sol_q(:)];
                send(pose_joints_pub, ros_msg);
                % sol_q(:)
            end
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Step 2: receive real hand pose from MuJoCo and update
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if ros_on && if_contacts_feedback && ~isempty(contacts_sub.LatestMessage)
                fprintf('(1) wait until Mujoco has reached the goal.\n')
                pause(1)
                while 1
                    mujoco_data = contacts_sub.LatestMessage.Data'; % (1, 7+16+12+12+4+1+3*n)
                    flag_reached_goal = mujoco_data(52);
                    if flag_reached_goal
                       break; 
                    end
                    pause(0.005);
                end
                fprintf('(2) Done!\n')
            
                sol_handHT_ = eye(4); % (4,4)
                sol_handHT_(1:3, 1:3) = quat2rotm(mujoco_data(4:7));
                sol_handHT_(1:3,4) = mujoco_data(1:3) * 1000;
                
                sol_q_ = mujoco_data(8:23); % (1,16)
                sol_positions_ = reshape(mujoco_data(24:35)*1000, [3,4]); % (3,4)
                sol_normals_ = reshape(mujoco_data(36:47), [3,4]); % (3,4)
                current_contact_label = mujoco_data(48:51); % 0: in contact; 1: not contacted. label assigned according to GPR convention
                hand.current_contact_label = current_contact_label(:);
                hand.current_fingertips = reshape(mujoco_data(53:64)*1000, [3,4]); % (3,4) the semi-sphere center of fingertips 
                
                
                %%% Cases, where bringing in perturbation is desired (add noise to vecT)
                % Case 1: finger not in contact
                if_perturbation = boolean(current_contact_label); % (1,4), 1 exists, indicating non-contacted points, add noise is true
                
%                 fprintf('Set perturbation based on contact: \n');
%                 disp(if_perturbation);
                
                % Case 2: finger stuck in local minimum region (latest N points are gatheres in a small circle)
                % Register latest contacts
                for iF = 1:nFingers
                    % check contact points of each finger, if finger in contact (label==0), then register the contacted point to the list
                    if current_contact_label(iF) == 0 % not in contact

                        old_cluster_center = mean(latestContacts(:,iF,:),3); % mean taken along the 3rd dimension

                        latestContacts(:,iF,1:end-1) = latestContacts(:,iF,2:end); % shift
                        latestContacts(:,iF,end) = sol_positions_(:,iF); % save the latest contact

                        new_cluster_center = mean(latestContacts(:,iF,:),3);

                        d = norm(old_cluster_center - new_cluster_center); % distance from new sampled point to old cluster center
                        if d < 5 % in mm % threshould of getting stuck into local minimum
                            fprintf('Finger %d, distance: %d, set perturbation true.\n', iF, d);
                            if_perturbation(iF) = true;
                        end
                    end
                end
                
%                 fprintf('Set perturbation after d: \n');
%                 disp(if_perturbation);
                
                contacts_one_step = reshape(mujoco_data(65:end), 3, []); % (3, n)
                hand.contacts_one_step = contacts_one_step * 1000;
                
                if all(sum(sol_positions_.^2, 1)) % if all the contacts are valid
                    sol_handHT = sol_handHT_;
                    sol_q = sol_q_;
                    sol_positions = sol_positions_;
                    sol_normals = sol_normals_;
                else
                    fprintf('some fingers never have contacts.\n');
                    disp(sol_positions_);
                end
            end            
            real_handHT = sol_handHT;
            real_q = sol_q;
            real_positions = sol_positions;
            real_normals = sol_normals;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Step 3: Update hand parameters %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            hand.T = real_handHT; % (4,4)
            hand.q = real_q; % (1,16)
            
            hand.contacts.moving_direction = normalize(real_positions - hand.contacts.positions, 1, 'norm');
            
            hand.contacts.positions = real_positions; % Current contact positions
            hand.contacts.normals = real_normals; % Current normal directions
            
            
            
            real_rotmH = real_handHT(1:3,1:3);
            real_posH = transpose(real_handHT(1:3,4));
            real_quatH = rotm2quat(real_rotmH); % (1,4), rotation matrix to hand quaternion
            
            for iF = 1:nFingers % Extract initial values of alpha and phi
                if ros_on
                    [alp, phi, ~] = solveForCylindricalCoordinates(hand, iF, real_positions, X_key, params, real_q, real_posH, real_quatH);
                else
                    alp = sol_alp(iF);
                    phi = sol_phi(iF);
                end
                
                hand.F{iF}.Link{end-1}.contact.crdCyl.alp = alp; % not used
                hand.F{iF}.Link{end-1}.contact.crdCyl.phi = phi;
                
                hand.F{iF}.q = hand.q(hand.qin==iF);
            end
            %{
            if ros_on
                % Solve for cylindrical coordinates of each contact
                % symvar(sym_contacts.positions):
                % [ alp14, alp24, alp34, alp44, phi14, phi24, phi34, phi44,...
                % q11, q12, q13, q14, q21, q22, q23, q24, q31, q32, q33, q34, q41, q42, q43, q44,
                % qH1, qH2, qH3, qH4,
                % xH, yH, zH]
                
                symContact = subs(sym_contacts.positions, X_key(params.idx_q), real_q); % (3,4)
                symContact = subs(symContact, X_key(params.idx_posH), real_posH); % [ xH, yH, zH]
                symContact = subs(symContact, X_key(params.idx_quatH), real_quatH); % [ qH1, qH2, qH3, qH4]
            end
            
            % Solve for alp, phi, rho of each contact
            for iF = 1:nFingers % Extract initial values of alpha and phi
                if ros_on
                    iSymContact = symContact(:,iF); % symvar: [ alpi4, phii4]
                    enqx = iSymContact(1) == real_positions(1);
                    enqy = iSymContact(2) == real_positions(2);
                    enqz = iSymContact(3) == real_positions(3);
                    var_alp = ['alp',num2str(iF),'4']; % on the 4th link
                    var_phi = ['phi',num2str(iF),'4'];
                    S = solve([enqx,enqy,enqz], {var_alp,var_phi}, 'Real',true);
                    i_real_alp = S.(var_alp);
                    i_real_phi = S.(var_phi);
                else
                    i_real_alp = sol_alp(iF); % sol_alp: (1,4)
                    i_real_phi = sol_phi(iF); % sol_phi: (1,4)
                end
                
                hand.F{iF}.Link{end-1}.contact.crdCyl.alp = i_real_alp;
                hand.F{iF}.Link{end-1}.contact.crdCyl.phi = i_real_phi;
                
                hand.F{iF}.q = hand.q(hand.qin==iF);
            end
            %}
            
            hand = mySGmoveHand(hand); % move hand to new configuration
            % mySGplotHand(hand);
            % title('Hand at desired exploration configuration');
            % hold off;
            
        else
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Failure Type I: Huge Error %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % The max. distance to object point cloud is larger than 20 mm,
            % indicating huge aberration from exploration results to
            % desired target, mark as failure

            % Change flag, mark this trial as failure
            fprintf('[Warning] Solved positions deviate from groundtruth, max dist: %d\n', max(dist_to_groundtruth));
            % exitflag = -1;
            % pause;
            
            hand.contacts.positions = this_contacts.positions;
            hand.contacts.normals = this_contacts.normals;
        end
        
    else
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Failure Type II: No Solution %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [c_sol, ceq_sol] = optExplore_nonlcon(X_sol, next_contacts, baseMatrices, objectValues, init_quatH);
        fprintf('No solutions found. Violated constraints: c: %d, ceq: %d:\n', sum(c_sol>=0), sum(ceq_sol~=0));
        hand.contacts.positions = this_contacts.positions;
        hand.contacts.normals = this_contacts.normals;
    end

    % fprintf('fmincon Completed.\n');
end