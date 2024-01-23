function link = updateLink(link, HT_this, HT_next, idx, lactv, crdCyl)
% Update values of the fields in link. Symbolic form remain untouched.
% See 'makeLink'.

    if nargin < 2 % create a virtual link
        HT_next = HT_this; % in case requested to plot the point on the joint
    end
    
    p_this = HT_this(1:3,4); % the reference frame of the ith link (fixed, not affected by the rotation of ith joint, equals to the 'HT_next' of the previous link)
    p_next = HT_next(1:3,4); % body reference frame, z direction is the axis of the link cylinder, affected by the rotation of the ith joint

    %% Update fields of link
    link.HT_this = HT_this;
    link.HT_next = HT_next;
    
    R = HT_next(1:3,1:3); % NOTICE THAT USE THE NEXT_LINK AS ORIENTATION REFERENCE
    link.p = p_this;
    link.r = R(:,1); % X direction is the radial direction
    link.n = R(:,2); % Y direction is the normal direction (pointing palm)
    link.L = norm(p_next-p_this); % constant
    
    if nargin >= 4
        if ~isequal(link.idx, idx)
            link.idx = idx;
            error('Link idx error.');
        end
    end
    
    if nargin >= 5 % Update link active status
        if ~isequal(link.lactv, lactv)
            link.lactv = lactv;
        end
    end
    
    if nargin >= 6 % Cylindrical coordinates are given. Used to update contact coordinates in link.
        if link.is_real
            link.contact = calcContactPoint(link, crdCyl);
        else
            error('Ask to update contact for an unreal link.');
        end
    end
end