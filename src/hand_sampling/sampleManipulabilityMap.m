%% This function samples the manipulability map of the finger link (currently fingertip).
% Only need to sample two fingers: thumb and one other finger.

function FeatureModel = sampleManipulabilityMap(hand, iF)

	sampling_approach = 'linspace';
	add_noise = true;
    stepLength = 10; % 10 degrees step length
    linSpace = 10; % 5 points for each joint

	%% Construct symbolic expression of Jacobian
	finger = hand.F{iF};
	fingertip = finger.Link{end};
	symFK = fingertip.symbolic.HT_next(1:3,4); % This is the fingertip (EE reference frame)
	Q = sym(strcat('q',num2str(iF),'%d'), [1,4]); % symbolic variables
	assert(isequal(symvar(symFK),Q));
	symJ = jacobian(symFK,Q); % (3,4)
	
	%% Sampling in joint space
	nJoints = hand.F{iF}.n;

	% Create meshgrid in joint space
	q_dim = cell(1,nJoints);
    
    switch sampling_approach
        case 'stepsize'
            delta_q = deg2rad(stepLength);
            for d = 1:nJoints % number of dof for iF-th finger
                q_dim{d} = finger.lb(d):delta_q:finger.ub(d); % sample from lower bound to upper bound
                if add_noise
                    q_dim{d} = q_dim{d} + normrnd(0,0.01,size(q_dim{d},1),size(q_dim{d},2));
                end
                delta_q = delta_q * 1.2; % attenuation factor for joints further from the finger base
            end
        case 'linspace'
            for d = 1:nJoints
                q_dim{d} = linspace(finger.lb(d), finger.ub(d), linSpace); % 10 samples uniformly distributed in the motion range of joint angle
                if add_noise
                    q_dim{d} = q_dim{d} + normrnd(0,0.01,size(q_dim{d},1),size(q_dim{d},2));
                end
            end
        otherwise
            error('NotImplementedError.');
    end
    
    q_mesh = cell(1,nJoints); % output meshgrid, each cell is value of one dimension
    [q_mesh{:}] = ndgrid(q_dim{:});
    num_q = numel(q_mesh{1}); % number of all q-values to sample
    fprintf('Total number of joint space samples: %d\n', num_q);

    % previousMap = []; % map of the previous joint
	rMap = zeros(num_q,3); % this is the reachability map of fingertip
	omegaMap = zeros(num_q,1);
    jointAngles = zeros(num_q,4);

    for idx = 1:num_q % iterate over all possible joint-value-combinations
        fprintf('Sample: %d - %d\n', num_q, idx);
        q_test = zeros(1,nJoints); % current q to sample
        for d = 1:nJoints % assign joint value of each DOF
            t = q_mesh{d};
            t = t(:); % [todo] Simplify
            q_test(d) = t(idx);
        end

        % rMap
        numFK = double(subs(symFK, Q, q_test));
        rMap(idx,:) = numFK; % (N,3)

        % omegaMap
        numJ = double(subs(symJ, Q, q_test));
        omega = sqrt(det(numJ*transpose(numJ)));
        omegaMap(idx) = omega; % (N,1)
        
        % save jointAngles
        jointAngles(idx,:) = q_test;
    end
    
    %% Obtain the isotropic reachability index
    isoMap = sampleReachabilityDistanceMap(rMap);

    %% Save local file
    save(strcat('F',num2str(iF),'L',num2str(fingertip.idx),'_FeatureModel.mat'),'rMap','omegaMap','jointAngles','isoMap');
    
    if nargout > 0
        FeatureModel = struct('rMap',rMap,... % (N,3) 3d positions
            'omegaMap',omegaMap,... % (N,1) manipulability index
            'jointAngles',jointAngles,... % (N,4) joint angles
            'isoMap',isoMap); % (N,1) isotropic index
    end

	%% Visualization
    %{
    figure, hold on;
    sz = 20;
    scatter3(rMap(:,1),rMap(:,2),rMap(:,3),sz,omegaMap(:));
    axis equal;
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Manipulability Index Values');
    hold off;
    %}
    
end