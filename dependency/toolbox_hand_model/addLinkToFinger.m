% Add 'Link' field to finger structure (update_link = false), or update link fields of the finger (update_link = true).

function finger = addLinkToFinger(finger, update_link)
    if nargin < 2
        update_link = false; % Create new link by default
    end
    
    if ~isfield(finger,'Link')
        finger.Link = {}; % link of fingers. Joint locates at the head (starting point) of a link.
        finger.idx_real_link = []; % (n+1,1), list to save index of real link (length>0), 1 is real, 0 is virtual, (n+1)th is the fingertip
    end
    finger.joints_pos = zeros(3,finger.n+1);

    % Update kinematic chain of finger
    referenceJoint = finger.base; % 'referenceJoint' is a HT transform mtx. Starting from base coordinate frame
    for k = 1:finger.n % k: idx of link. iterate over number of joints on the finger
        % Virtual link and joint number has 1-1 correspondance
        refJoint_old = referenceJoint; % the old ref on the base
        localTransf = mySGDHMatrix(finger.DHpars(k,:));
        referenceJoint = referenceJoint*localTransf; % the new ref locates on the tip of the link
        finger.joints_pos(:,k) = referenceJoint(1:3,4); % Cartesian position of l^{th} joint

        % Notice that: referenceJoint ('HT_next') is the HT after rotating
        % the n^{th} joint. So can consider the joint locates at the base
        % of the link; 'HT_this' is not affected by the rotation; while
        % 'HT_next' is affected by the rotation of the link.
        if update_link
            finger.Link{k} = updateLink(finger.Link{k}, refJoint_old, referenceJoint, k, finger.contacted_link(k));
        else
            finger.Link{k} = makeLink(refJoint_old, referenceJoint, k, finger.contacted_link(k), finger.cc); % set base as the reference point of the link
            finger.idx_real_link(end+1) = finger.Link{k}.is_real;
        end
    end
    
    % Each column of finger.tip: [x-direction at the finger tip coordinate frame, y, z, position]
    finger.tip = referenceJoint; % [deprecated] use 'finger.tip = finger.Link{end}' after initialization, instead
    finger.joints_pos(:,end) = referenceJoint(1:3,4);
    
    if update_link % The last link, use the same HT_this and HT_next
        finger.Link{k+1} = updateLink(finger.Link{k+1}, referenceJoint, referenceJoint, k+1, finger.contacted_link(k+1));
    else        
        finger.Link{k+1} = makeLink(referenceJoint, referenceJoint, k+1, finger.contacted_link(k+1), finger.cc); % fingertip as the last link (virtual link with 0 length)
        finger.idx_real_link(end+1) = finger.Link{k+1}.is_real;
    end
end