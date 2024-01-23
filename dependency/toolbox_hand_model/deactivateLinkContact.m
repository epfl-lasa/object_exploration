% This function set one specified link on one specified finger to be inactive.
% - It updates the corresponding parameters in hand struct
% 
% Input:
%     * hand: hand struct
%     * idx_fngr: index of finger in the hand
%     * idx_lnk: index of link in the finger

function [hand, link] = deactivateLinkContact(hand, idx_f, idx_l)

switch nargin
    
    case 3 % deactivate one link, specified by hand-finger-link
        if ~idx_f || ~idx_l % palm
            hand.pavtc = 0;
        else
            hand.F{idx_f}.Link{idx_l}.lactv = 0;

            if hand.F{idx_f}.Link{idx_l}.is_real % 'contact' only exits for real link
                hand.F{idx_f}.Link{idx_l}.contact.cactv = 0; % set contact point on the link as active
            end

            nq_pre = min([idx_l, hand.F{idx_f}.n]); % number of joints ahead of (include idx_l) idx_lnk, F.n is the number of joints on the finger. Incase idx_lnk > finger.n (possible for fingertip link)
            hand.F{idx_f}.active_joints(1:nq_pre) = 0; % deactivate current (index: idx_l) and all previous joints

            q_idx = find(hand.qin == idx_f, 1); % first non-zero position (starting point index) of the indices of all joints of the finger in the hand
            hand.qactv(q_idx:q_idx+nq_pre-1) = 0; % deactivate the joint angles
        end
        
    case 2 % deactivate all links on one specified finger
        if ~idx_f % palm
            hand.pavtc = 0;
        else
            for idx_l = 1:numel(hand.F{idx_f}.Link)
                hand.F{idx_f}.Link{idx_l}.lactv = 0;
                if hand.F{idx_f}.Link{idx_l}.is_real
                    hand.F{idx_f}.Link{idx_l}.contact.cactv = 0;
                end
            end
            hand.F{idx_f}.active_joints = zeros(size(hand.F{idx_f}.active_joints));
            q_idx = find(hand.qin == idx_f, 1); % first non-zero position (starting point index) of the indices of all joints of the finger in the hand
            hand.qactv(q_idx:q_idx+hand.F{idx_f}.n-1) = 0; % deactivate the joint angles

            hand.factv(idx_f) = 0; % index of active fingers
        end
        
    case 1 % deactivate all links of all fingers of the hand
        hand.pactv = 0; % deactivate palm
        for idx_f = 1:numel(hand.F)
            for idx_l = 1:numel(hand.F{idx_f}.Link)
                hand.F{idx_f}.Link{idx_l}.lactv = 0;
                if hand.F{idx_f}.Link{idx_l}.is_real
                    hand.F{idx_f}.Link{idx_l}.contact.cactv = 0;
                end
            end
            hand.F{idx_f}.active_joints = zeros(size(hand.F{idx_f}.active_joints));
            q_idx = find(hand.qin == idx_f, 1); % first non-zero position (starting point index) of the indices of all joints of the finger in the hand
            hand.qactv(q_idx:q_idx+hand.F{idx_f}.n-1) = 0; % deactivate the joint angles
            
            hand.factv(idx_f) = 0; % index of active fingers
        end
end

if nargout>1
    link = hand.F{idx_f}.Link{idx_l};
end
end