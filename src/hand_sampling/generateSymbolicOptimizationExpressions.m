% This function calculates symbolic expressions used in the main code and
% saves expressions as local .m files. Including:
% - sym_contacts.positions: (3,4), contact point position
% - sym_contacts.normal: (3,4), normal direction on contact point
% - sym_nonl_c: nonlinear inequality constraints
% - sym_nonl_ceq: nonlinear equality constraints
% - sym_obj_fun: objective function

function [c, ceq, obj] = generateSymbolicOptimizationExpressions(hand, params)
    global path_symbolic
    global ifGenSymEqCon ifGenSymIneqCon ifGenSymObjfun
    global sym_contacts
    global testObjectiveFunction
    
    global abductionIdx extensionIdx
    
    global jointWeightMatrix slackWeightMatrix

    optimizeHandPose = params.optimizeHandPose; % hand pose adaptation
    optimizeFingers = params.optimizeFingers; % finger kinematic property improvement
    
    if nargin < 3
        if isempty(sym_contacts)
            try
                sym_contacts_file = load('sym_contacts.mat');
                sym_contacts = sym_contacts_file.sym_contacts;
            catch
                error('sym_contacts.mat does not exist.');
            end
        end
    end

    nFingers = hand.n;
    sym_positions = sym_contacts.positions; % (3,4)
    sym_normals = sym_contacts.normals; % (3,4)
    
    % global next_positions next_normals
    next_positions = transpose(sym('np%d%d',[4,3])); % (3,4)
    next_normals = transpose(sym('nn%d%d',[4,3])); % (3,4)
    
    X_key = params.X_key;
    X0_sym = params.X0_sym; % (1,43)
    delta_X = X_key - X0_sym; % (1,43)

    VecT_sym = [sym('vt%dx',[1,4]);sym('vt%dy',[1,4]);sym('vt%dz',[1,4])]; % (3,4)

    sym_T = params.sym_T;
    sym_posH = params.sym_posH; % xH, yH, zH
    sym_quatH = params.sym_quatH;
    sym_slack = params.sym_slack; % (1,12),[delta11 (X), delta12 (Y), delta13 (Z), delta21, ..., delta43]

    sym_object.center = sym({'ox','oy','oz'}); % corresponding numerical values: objectValues
    sym_object.dist.min = sym('dmin');
    
    quat_old = sym({'qH1_old','qH2_old','qH3_old','qH4_old'});
    
    Q_general = X_key(~params.idx_slack); % all variables are used except slack variables
    delta_Q_general = delta_X(~params.idx_slack);
    
    fbMtx = sym('fb%d%d%d',[4,4,nFingers]); % Correspond to: `baseMatrices.fingers`
    pbMtx = sym('pb%d%d',[4,4]); % Correspond to: `baseMatrices.palm`

    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Symbolic Nonlinear Equality Constraints %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if ifGenSymEqCon
        ceq = sym([]); % symbolic nonl. equality constraints

        %%% I. Quaternion norm
        if optimizeHandPose
            ceq(end+1) = sym_quatH*transpose(sym_quatH) - 1;
        end

        %%% II. desired contact position, as soft constraint
        Eq = sym_positions - next_positions + reshape(sym_slack,[3,4]); % (3,4)
        ceq(end+1 : end+numel(Eq)) = reshape(Eq,1,[]);

        %%% III. motion direction is perpenticular to surface normal
        %{
        global this_positions this_normals
        this_positions = transpose(sym('tp%d%d',[4,3])); % (3,4) % Each column is the contact coordinate of one fingertip: [ n11, n21, n31, n41; n12, n22, n32, n42; n13, n23, n33, n43]
        this_normals = transpose(sym('tn%d%d',[4,3])); % (3,4)
        motionVector = sym_positions - this_positions; % (3,4)
        ceq(end+1 : end+nFingers) = dot(motionVector,next_normals); % (1,4) each finger is perpendicular to its surface normal
        %}

        %%% Final: save local function file
        if optimizeHandPose
            % No need to subs symvars if hand pose is controlled
        else
            ceq = subs(ceq, sym_quatH, [1,0,0,0]);
            assert(~any(ismember(sym_quatH, symvar(ceq))));
        end
        
        matlabFunction(ceq,'File',fullfile(path_symbolic,'nonl_ceq.m'),...
            'Vars',{X_key,next_positions,next_normals},...
            'Optimize',false);
        fprintf('Saved: Nonlinear Equality Constraints.\n');
    end
    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Symbolic Nonlinear Inequality Constraints %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if ifGenSymIneqCon
        c = sym([]); % symbolic nonl. inequality constraints
        % Mainly collision-related constraints [todo]

        %%% distance from palm to object center must larger than the min.
        % distance from object center to object surface
        %{
        if optimizeHandPose
            distVec = sym_posH - sym_object.center; % (1,3)
            c(end+1) = sym_object.dist.min^2 - distVec*transpose(distVec);
        end
        %}

        %%% Force finger base positions to be far from object surface (previously: Eq. (21))
        if optimizeHandPose
            for iF = 1:nFingers
                baseF = sym_T * fbMtx(:,:,iF);
                base_point = baseF(1:3,4);
                distVec = transpose(base_point) - sym_object.center; % (1,3)
                c(end+1) = sym_object.dist.min^2 - distVec*transpose(distVec);
            end
        end

        %%% Orientation of palm: force palm normal direction to face object surface
        %%% [Consider soft constraint]
        if optimizeHandPose
            palm_base = sym_T * pbMtx; % palm_base = sym_T * hand.P.palm_base;
            palm_center = palm_base(1:3,4); % (3,1)

            palm_normal = -palm_base(1:3,3); % (3,1), pointing downwards
            Vec = sym_object.center(:) - palm_center(:);
            % (3,1) [outdated] Object center need to be changed: replace Vec with the local surface normal direction
            % Update: sym_object.center(:) is replaced by the center of ROI, as using the local surface normal directions are computationally expensive

            squared_norm_palm_normal = transpose(palm_normal) * palm_normal;
            squared_norm_vec = transpose(Vec) * Vec;

            deltaSimilarity = ((transpose(palm_normal)*Vec)*(transpose(palm_normal)*Vec))/(squared_norm_palm_normal * squared_norm_vec);
            c(end+1) = 0.8830 - deltaSimilarity; % cosd(20)^2 = 0.8830, larger than 
        end

        %%% Continuity of hand pose (restrict step-change in hand pose quaternion)
        if optimizeHandPose
            quat_conj_old = quat_old.*[1,-1,-1,-1];
            quat_multiply = sym_quatprod(quat_conj_old, sym_quatH); % quad multiply: quad_conj_old * quad_hand, the quaternion to transform from `quat_old` to current quaternion
            % tangent of half theta, theta is the rotation angle from the
            % old hand quaternion to the current hand quaternion
            % quat(1) = cos(theta/2)
            % quat(2:4) = sin(theta/2)*e, e is the unit axis of rotation
            squared_tan_half_theta = (quat_multiply(2:4)*transpose(quat_multiply(2:4))) / quat_multiply(1).^2; % tangent of half-rotation angle, 2 * atan2(norm(quad_multiply), quad_transfer(1));

            % c(end+1) = -0.5774 + tan_half_theta; % < 60 deg (tan_half_theta < tand(30))
            % c(end+1) = -0.5774 - tan_half_theta; % >-60 deg (tan_half_theta >-tand(30))
            % c(end+1) = squared_tan_half_theta - 0.7854.^2;
            c(end+1) = squared_tan_half_theta - 0.5774.^2;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%
            % Force palm not to flip %
            %%%%%%%%%%%%%%%%%%%%%%%%%%
            % The angle included in the Z-axis of palm RF and the Z-axis of
            % World CF does not exceed pi/2. Leading to an interval of 'w'.
            % c(end+1) = abs(sym_quatH(1)) - 1; % quaternion in MATLAB: (w,x,y,z)
            % c(end+1) = sqrt(2)/2 - abs(sym_quatH(1));
            c(end+1) = 1 / 2 - abs(sym_quatH(1));  % to make the angle within [0, 120] degrees
        end

        %%% Collision avoidance btw adjacent fingertips
        Nx = 3; % Use 3 samples on each link
        Ny = 3;
        lc = length(c);
        
        % fprintf('Length of c before collision avoidance: %d\n', length(c));
        for iF = 1:nFingers-1
            iFinger = hand.F{iF};
            jFinger = hand.F{iF+1};
            
            for iL = 1:2 % Consider last but 2, 1 (3rd, 4th link of finger)
                iFingerTip = iFinger.Link{end-iL};
                assert(iFingerTip.is_real);
                x1 = iFingerTip.symbolic.HT_this(1:3,4);
                x2 = iFingerTip.symbolic.HT_next(1:3,4);

                jFingerTip = jFinger.Link{end-iL};
                assert(jFingerTip.is_real);
                y1 = jFingerTip.symbolic.HT_this(1:3,4);
                y2 = jFingerTip.symbolic.HT_next(1:3,4);

                dist = distanceLineSegments(x1,x2,y1,y2,Nx,Ny,'paired');
                assert(all(ismember(symvar(dist),X_key)));

                minDistance = iFingerTip.radius + jFingerTip.radius;
                minDistance = minDistance * 1.5; % increase the collision distance so that it is less likely to have collision
                
                c(end+1 : end+numel(dist)) = minDistance.^2 - dist; % Use squared distance, do not use vector norm
            end
        end
        % fprintf('Length of c after collision avoidance: %d\n', length(c));
        fprintf('Number of collision avoidance constraints: %d\n', length(c)-lc);

        % Save constraints to local file
        if optimizeHandPose
            % matlabFunction(c,'File',fullfile(path_symbolic,'nonl_c.m'),'Vars',{X_key, fbMtx, pbMtx, sym_object.center, sym_object.dist.min},'Optimize',false);
            % fprintf('Saved: Nonlinear Inequality Constraints.\n');
        else
            c = subs(c, sym_quatH, [1,0,0,0]);
            % assert(~any(ismember(sym_quatH, symvar(c))));
            % matlabFunction(c,'File',fullfile(path_symbolic,'nonl_c.m'),'Vars',{X_key, sym_object.center, sym_object.dist.min},'Optimize',false);
            % fprintf('Saved: Nonlinear Inequality Constraints.\n');
        end

        matlabFunction(c,'File',fullfile(path_symbolic,'nonl_c.m'),...
            'Vars',{X_key, fbMtx, pbMtx, sym_object.center, sym_object.dist.min, quat_old},...
            'Optimize',false);
    end
    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Symbolic Objective Function %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if ifGenSymObjfun
        %%% (2) Hand joints damping term
        sym_q = params.sym_q; % (1,16)

        % delta_q = sym_q; % force to close to 0
        % delta_q = sym_q - transpose(hand.q); % only penalize the change
        
        delta_q = sym_q; % (1,16)
        delta_q(extensionIdx) = delta_q(extensionIdx) - transpose(hand.limit(extensionIdx,1)); % prefer fingers to open (move towards limit(:,1), fully open)
        
        diag_M_q = ones(1,numel(delta_q));
        
        diag_M_q(1:4:end) = jointWeightMatrix(1); % 1st DoF of each finger, extension/flexion of thumb, and ab-/adduction of fingers
        diag_M_q(2:4:end) = jointWeightMatrix(2); % ab-/adduction of thumb, and flexion/extension of fingers
        diag_M_q(3:4:end) = jointWeightMatrix(3); % flexion/extension
        diag_M_q(4:4:end) = jointWeightMatrix(4); % flexion/extension
        
        % Correction for thumb
        diag_M_q(1) = jointWeightMatrix(2); % flexion of thumb
        diag_M_q(2) = jointWeightMatrix(1); % rotation of thumb
        
        M_q = diag(diag_M_q); % coefficients for damping of joint angles
        
        dampingJoints = transpose(delta_q(:)) *M_q *delta_q(:);
        %{
        abductionJoints = params.sym_q(1:4:end);
        M_q = eye(numel(abductionJoints));
        dampingJoints = transpose(abductionJoints(:))*M_q*abductionJoints(:);
        %}

        %%% (3) Hand pose damping term
        %{
        handquat = rotm2quat(hand.T(1:3,1:3));
        delta_quat = reshape(sym_quatH,1,[]) - reshape(handquat,1,[]);
        dampingQuat = transpose(delta_quat(:))*delta_quat(:);
        %}

        %%% (4) Squared sum of slack variables
        sym_slack = params.sym_slack;
        
        % M_s = eye(numel(sym_slack));
        % Slack variables: [delta11 (F1-x), delta12 (F1-y), delta13 (F1-z), delta21, ..., delta43]

        diag_Ms = ones(1,numel(sym_slack)); % (1,12)
        diag_Ms(1:3) = slackWeightMatrix(1); % thumb target, (x,y,z)
        diag_Ms(4:6) = slackWeightMatrix(2); % index target
        diag_Ms(7:9) = slackWeightMatrix(3);
        diag_Ms(10:12) = slackWeightMatrix(4);
        M_s = diag(diag_Ms);

        dampingSlack = transpose(sym_slack(:))*M_s*sym_slack(:);

        if optimizeFingers
            %%% (6) Increase isotropy value of fingers and thumb
            iFT = load('isotropyFunctionThumb.mat'); % The expression of isotropic value of thumb, expressed using thumb joint angles
            iFI = load('isotropyFunctionIndex.mat');
            iFM = load('isotropyFunctionMiddle.mat');
            iFR = load('isotropyFunctionRing.mat');

            iFT = iFT.isotropyFunctionThumb;
            iFI = iFI.isotropyFunctionIndex;
            iFM = iFM.isotropyFunctionMiddle;
            iFR = iFR.isotropyFunctionRing;
            
            % if testObjectiveFunction
            isotropyValues = 1/iFT + 1/iFI + 1/iFM + 1/iFR;
            % else
                % isotropyValues = iFT + iFI + iFM + iFR;
                % isotropyValues = 1/isotropyValues; % Need to be minimized
            % end

            %%% (7) Increase manipulability of fingers and thumb
            oFT = load('omegaFunctionThumb.mat');
            oFI = load('omegaFunctionIndex.mat');
            oFM = load('omegaFunctionMiddle.mat');
            oFR = load('omegaFunctionRing.mat');
            
            oFT = oFT.omegaFunctionThumb;
            oFI = oFI.omegaFunctionIndex;
            oFM = oFM.omegaFunctionMiddle;
            oFR = oFR.omegaFunctionRing;
            
            % if testObjectiveFunction
            omegaValues = 1/oFT + 1/oFI + 1/oFM + 1/oFR;
            % else
            %     omegaValues = oFT + oFI + oFM + oFR;
            %     omegaValues = 1/omegaValues;
            % end

            % obj = 4*dampingJoints + 16*dampingSlack + isotropyValues + omegaValues; % new
            obj = 2.5*dampingJoints + 10*dampingSlack + isotropyValues + omegaValues; % old
            
        else
            obj = 2.5*dampingJoints + 10*dampingSlack;
        end

        %%% (8) Information gain [Farshad][Test]
        %{
        if testObjectiveFunction
            thumbTip = sym_positions(:,1);
            indexTip = sym_positions(:,2);
            middleTip = sym_positions(:,3);
            ringTip = sym_positions(:,4);
            J1 = jacobian(thumbTip,  Q_general); % (3,31), [alp14, phi14, q11, q12, q13, q14, qH1, qH2, qH3, qH4, xH, yH, zH]
            J2 = jacobian(indexTip,  Q_general); % (3,31)
            J3 = jacobian(middleTip, Q_general); % (3,31)
            J4 = jacobian(ringTip,   Q_general); % (3,31)
            J_block = [J1;J2;J3;J4]; % (12,31)
            VecT_row = reshape(VecT_sym,1,[]); % [ vt1x, vt1y, vt1z, vt2x, vt2y, vt2z, vt3x, vt3y, vt3z, vt4x, vt4y, vt4z], tangential directions of gradient
            varGradientTerm = VecT_row * J_block * transpose(delta_Q_general);

            obj = obj - varGradientTerm;
        end
        %}

        if optimizeHandPose
            % assert(all(ismember(symvar(obj),X_key)));
        else
            obj = subs(obj, sym_quatH, [1,0,0,0]);
            % assert(~any(ismember(sym_quatH,symvar(obj))));
            % assert(all(ismember(symvar(obj),X_key)));
        end
        
        matlabFunction(obj,'File',fullfile(path_symbolic,'objfun.m'),'Vars',{X_key,X0_sym,VecT_sym},'Optimize',false);
        fprintf('Saved: Objective Function.\n');
        
        % Construct objective function for later evaluation
        if exist('dampingJoints','var')
            matlabFunction(dampingJoints,  'File',fullfile(path_symbolic,'objterm_joints.m'),  'Vars',{X_key},'Optimize',false);
        end
        if exist('dampingSlack','var')
            matlabFunction(dampingSlack,   'File',fullfile(path_symbolic,'objterm_slack.m'),   'Vars',{X_key},'Optimize',false);
        end
        if exist('isotropyValues','var')
            matlabFunction(isotropyValues, 'File',fullfile(path_symbolic,'objterm_isotropy.m'),'Vars',{X_key},'Optimize',false);
        end
        if exist('omegaValues','var')
            matlabFunction(omegaValues,    'File',fullfile(path_symbolic,'objterm_omega.m'),   'Vars',{X_key},'Optimize',false);
        end
        if exist('varGradientTerm','var')
            matlabFunction(varGradientTerm,'File',fullfile(path_symbolic,'objterm_variance.m'),'Vars',{X_key,X0_sym,VecT_sym},'Optimize',false);
        end
        
        fprintf('Saved: Individual terms of objective function.\n');
        
    end
end


function n = sym_quatprod(q,r)
    % Calculate multiplication of symbolic quaternions, q*r
    % q = q0 + i*q1 + j*q2 + k*q3;
    % r = r0 + i*r1 + j*r2 + k*r3;
    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
    r0 = r(1); r1 = r(2); r2 = r(3); r3 = r(4);

    n0 = r0*q0 - r1*q1 - r2*q2 - r3*q3;
    n1 = r0*q1 + r1*q0 - r2*q3 + r3*q2;
    n2 = r0*q2 + r1*q3 + r2*q0 - r3*q1;
    n3 = r0*q3 - r1*q2 + r2*q1 + r3*q0;
    n = [n0, n1, n2, n3];
end