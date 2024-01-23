% This function solves hand inverse kinematics.
% 
% Input:
% - hand: hand model (Allegro Hand model for now)
% - fingerList: list of fingers need move
% - positionTgt: (nFinger, 3), desired contact position on the finger distal
% - normalTgt: (nFinger, 3), desired contact normal vector on finger distal
% 
% Output:
% - hand: hand model at target position
% - X_sol: the actual Cartesian position of the finger tip
% - Q_sol: the corresponding finger joint angles
% 
% Example:
% run:
%   solveHandIK();

function hand = solveHandIK(hand, fingerIndex, tgt_position, tgt_normal)
    clc;
    clear global;
    
    if nargin < 1
        hand_file = load('allegro_hand_model.mat');
        % which('allegro_hand_model.mat');
        hand = hand_file.hand;
    end
    nFingers = hand.n; % number of fingers
    
    if nargin < 2
        fingerIndex = ones(1,nFingers);
    end
    
    if nargin < 3
        % Test code by randomly assign target position for each finger
        percent = 30; % largest percent of next exploratory movement
        % tgt_position = zeros(nFingers, 3);
        tgt_position = transpose(hand.ftips); % use current fingertips positions
        tgt_position = tgt_position.*(1 + randi([-percent,percent],size(tgt_position))/1000); % [TODO] Esitimation of workspace to bound
        
        tgt_normal = zeros(size(tgt_position));
        for iF = 1:nFingers
            radial = hand.F{iF}.Link{end-1}.contact.symbolic.r; % radial direction of finger distal
            radial = double(subs(radial, symvar(radial), hand.F{iF}.q.'));
            radial = rotx(rad2deg(rand/6))*radial(:); % rotate around x for a random angle in [-pi/6, pi/6]
            tgt_normal(iF,:) = normalize(radial, 'norm');
        end
    end
    
    % figure, hold on;
    % mySGplotHand(hand);
    % scatter3(tgt_position(:,1), tgt_position(:,2), tgt_position(:,3), 100, 'r', 'filled');
    % hold off;

    % oldHand = hand;
    
    % For evaluation and comparison
    %{
    global Scores
    score_struct = struct('time',[],'fval',[],'pErr',[],'rErr',[]);
    Scores.fmincon = score_struct;
    Scores.qp = score_struct;
    %}
    
    for iF = 1:nFingers
        if fingerIndex(iF)
            fprintf('Finger index: %d\n', iF);
            
            % Solve IK using quadratic programming [Large numerical error, performance is not satisfactory.]
            % finger_qp = solveFingerIK_QP(hand.F{iF}, tgt_position(iF,:), tgt_normal(iF,:), 10);
            
            %% Solve IK using fmincon
            finger = solveFingerIK(hand.F{iF}, tgt_position(iF,:), tgt_normal(iF,:));
            
            hand.F{iF} = finger; % update hand model
            hand.ftips(:,iF) = finger.tip(1:3,4); % update hand position
            hand.q(hand.qin == iF) = finger.q; % [todo]: check collision among fingers during IK
        end
    end
    
    % Visualization of the final hand configuration
    hand = mySGmoveHand(hand, hand.q);
    figure, hold on;
    mySGplotHand(hand);
    title('IK Solution');
    trajectoryVisualization(hand, oldHand.q, hand.q, 10);
end