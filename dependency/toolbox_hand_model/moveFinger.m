function finger = moveFinger(finger, q, eval_type)
% This function is customized. No corresponding function in original SynGrasp package.
% 
% Original reference function: mySGmoveHand(hand,q)
% Move finger F to the new joint angles vector 'q'
% Input
%   * finger: single finger of a robotic hand
%   *       finger.active: a boolean vector indicating if the joint is active or not
%   * q: target joint position
%   * eval_type: evaluation type, 'sym' to use the symbolic expression, or 'num' to use the
%   numerical expression
% 
% Output
% * F: Finger with joints moved to desired q
    
    if nargin < 3 % set numerical command as default
        eval_type = 'num'; % 'eval_type' is true only to calc. reachability map based on symbolic form. Normal use of this function requires 'false'
    end
    if nargin < 2 % new finger joints are not given, just update the finger properties using current joint angles
        q = finger.q;
    else
        q = q(:); % force transform to column vector
    end
    
    if ~jointSafetyCheck(q, finger.lb, finger.ub)
        warning('moveFinger: Joint bounds violated.');
    end
    if ~isequal(length(q), finger.n)
        error('moveFinger: joint angle size does not match.');
    end
    
    %%% Update parameters
    q_old = finger.q;
    q(~finger.active_joints) = q_old(~finger.active_joints); % skip inactive joints by keeping the old values of the joint with 'active_joints' flag setting to 0

    finger.q = q;
    finger.DHpars(:,3) = q;
    
    %%% FOR ALLEGRO HAND [Allegro Hand DH Offset IN JOINT ANGLE VARIABLES]
    if strcmp(finger.hand_type, "AllegroHandLeft")
        % Add joint offset to the DH parameters
        % This is the offset of Allegro hand model. For details, refer to the
        % original SG package, function 'mySGmoveHand'.
        dh_offset_allegro_left = [0, 0, 0, 0;... % Each column corresponds to the offset of one finger
            pi/2, pi/2, pi/2, pi/2;...
            0, 0, 0, -pi/2;...
            0, 0, 0, 0];
        % The finger order has been changed in the model "mySGallegroLeft".
        % In the original SynGrasp definition, the order is: [index, middle, ring, thumb],
        % In the customized function, the order is: [thumb, index, middle ring].
        dh_offset_allegro_left = dh_offset_allegro_left(:,[4,1,2,3]);
        finger.DHpars(:,3) = finger.DHpars(:,3) + dh_offset_allegro_left(:,finger.idx);
    end
    if strcmp(finger.hand_type, "AllegroHandRight")
        dh_offset_allegro_right = [0, 0, 0, 0;...
            pi/2, pi/2, pi/2, pi/2;...
            0, 0, 0, pi/2;...
            0, 0, 0, 0];
        dh_offset_allegro_right = dh_offset_allegro_right(:,[4,1,2,3]);
        finger.DHpars(:,3) = finger.DHpars(:,3) + dh_offset_allegro_right(:,finger.idx);
    end
    
    
    %% Symbolic Evaluation of finger structure fields
    if isequal(eval_type,'sym')
        %%% THIS TYPE IS USED IN OPTIMIZATION ONLY.
        % evaluation type: symbolic, create sym expression
        % Obtain symbolic expression from base to each contact points on the link
        % Symbolic form exists. Just substitute values into the symbolic
        % form and obtain the numerical values.
        try
            % Q = sym(['q',num2str(finger.idx),'%d'],[1 finger.n]); % [qi1,qi2,qi3,qi4]
            qcell = num2cell(q.');

            finger.joints_pos = feval(['F',num2str(finger.idx),'_joints_pos'],qcell{:}); % obtain joint position values by evaluating function
            finger.tip = feval(['F',num2str(finger.idx),'_tip'],qcell{:}); % obtain finger tip numerical values by evaluating functions
            
            for k = 1:numel(finger.Link) % iterate over links
                finger.Link{k}.HT_this = feval(['F',num2str(finger.idx),'L',num2str(k),'_HT_this'],qcell{:});
                finger.Link{k}.HT_next = feval(['F',num2str(finger.idx),'L',num2str(k),'_HT_next'],qcell{:});
                finger.Link{k}.p = finger.Link{k}.HT_this(1:3,4); % see 'makeLink'
                finger.Link{k}.r = finger.Link{k}.HT_next(1:3,1);
                finger.Link{k}.n = finger.Link{k}.HT_next(1:3,2);
            end
        catch
            warning('Files do not exist.');
        end
    elseif isequal(eval_type,'num') % only evaluate the numerical expression
        %% Numerical Evaluation of finger structure fields
        % (1) Use numerical command, or (2) Symbolic form has not been constructed, suggesting initialization
        % Notice that # links not necessarily equals # joints, since joint
        % may have more than 1 dof. These joints result in `virtual links`
        % with 0 length.
        update_link = true; % Link already exists. Just update.
        finger = addLinkToFinger(finger, update_link);
    else
        error('Unexpcted case.');
    end
end