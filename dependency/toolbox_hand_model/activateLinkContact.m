% This function enables the contact on one specified link on one specified finger.
% - It sets all previous joints on the finger as active
% - It updates the corresponding parameters in hand struct
% 
% Input:
%     * hand: hand struct
%     * idx_f: index of finger in the hand
%     * idx_l: index of link in the finger
% Output:
% 	  * hand: updated hand struct
% 	  * HTcp: homogeneous transformation mtx from base (palm) to the contact point (cp). Contact point should be HTcp(1:3,4)

function [hand, link] = activateLinkContact(hand, idx_f, idx_l)

switch nargin
    
    case 3 % activate one link on the specified finger
        % determine the index of joints on the link base
        if ~idx_f || ~idx_l % palm
            hand.pavtc = 1;
        else
            hand.F{idx_f}.Link{idx_l}.lactv = 1;
            if hand.F{idx_f}.Link{idx_l}.is_real
                hand.F{idx_f}.Link{idx_l}.contact.cactv = 1;
                hand.F{idx_f}.Link{idx_l}.contact = calcContactPoint(hand.F{idx_f}.Link{idx_l}); % Update HTcp with the new crdCyl [Only in activateLinkContact function]
            end

            nq_pre = min([idx_l, hand.F{idx_f}.n]); % number of joints ahead of idx_lnk, in case idx_lnk > finger.n (possible for fingertip link)
            hand.F{idx_f}.active_joints(1:nq_pre) = 1; % activate current (index: idx_lnk) and all previous joints

            q_idx = find(hand.qin == idx_f, 1); % first non-zero position (starting point index) of the indices of all joints of the finger in the hand
            hand.qactv(q_idx:q_idx+nq_pre-1) = 1;
            hand.factv(idx_f) = 1; % set the finger as active in the hand
        end
        
    case 2 % activate one entire finger
        if ~idx_f % palm
            hand.pavtc = 1;
        else
            for idx_l = 1:numel(hand.F{idx_f}.Link)
                hand.F{idx_f}.Link{idx_l}.lactv = 1;
                if hand.F{idx_f}.Link{idx_l}.is_real
                    hand.F{idx_f}.Link{idx_l}.contact.cactv = 1;
                    hand.F{idx_f}.Link{idx_l}.contact = calcContactPoint(hand.F{idx_f}.Link{idx_l});
                end
            end
            hand.F{idx_f}.active_joints = ones(size(hand.F{idx_f}.active_joints));
            q_idx = find(hand.qin == idx_f, 1); % first non-zero position (starting point index) of the indices of all joints of the finger in the hand
            hand.qactv(q_idx:q_idx+hand.F{idx_f}.n-1) = 1; % deactivate the joint angles
            hand.factv(idx_f) = 1; % index of active fingers
        end
        
    case 1 % activate entire hand
        hand.pactv = 1; % activate palm
        for idx_f = 1:numel(hand.F)
            for idx_l = 1:numel(hand.F{idx_f}.Link)
                hand.F{idx_f}.Link{idx_l}.lactv = 1;
                if hand.F{idx_f}.Link{idx_l}.is_real
                    hand.F{idx_f}.Link{idx_l}.contact.cactv = 1;
                    hand.F{idx_f}.Link{idx_l}.contact = calcContactPoint(hand.F{idx_f}.Link{idx_l});
                end
            end
            hand.F{idx_f}.active_joints = ones(size(hand.F{idx_f}.active_joints));
            q_idx = find(hand.qin == idx_f, 1); % first non-zero position (starting point index) of the indices of all joints of the finger in the hand
            hand.qactv(q_idx:q_idx+hand.F{idx_f}.n-1) = 1; % deactivate the joint angles
            hand.factv(idx_f) = 1; % index of active fingers
        end
end

if nargout > 1
    link = hand.F{idx_f}.Link{idx_l}; % update hand
end

end