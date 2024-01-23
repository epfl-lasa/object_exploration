function link = makeLink(HT_this, HT_next, idx, lactv, crdCyl)
% Make the link, represented by the homogeneous transformation of the link base and the link tip (base of the next link).
% Input:
%     * HT_this: homogeneous transformation matrix from the base of finger to the base of the digit link
%     * HT_next: homogeneous transformation matrix from the base of finger
%     to the end of the digit link
%     * lactv: if the link is active (in contact)
%     * crdCyl: cylindrival parameters of the contact point, [rho, phi, alp]
%     % * type: type of the reference point: 'center' or 'base'
%     * idx: index to indicate which link it is in the finger
% Output: 
%     * link: struct, contains the following information of the link's reference point:
%       * link.p: position of the reference point
%       * link.n: normal vector of the reference point (pointing towards front/finger inner side)
%       * link.r: radial direction vector of the link (pointing from the base to the next base)
%       * link.L: length of the link

    if nargin < 4
        lactv = true; % if this link is in contact (1), and if so, update the contact point
    end
    if nargin < 3 % index of this link in the father finger
        idx = 0;
    end
    if nargin < 2 % create a virtual link
        HT_next = HT_this; % in case requested to plot the point on the joint
    end
    
    p_this = HT_this(1:3,4); % the reference frame of the ith link (fixed, not affected by the rotation of ith joint, equals to the 'HT_next' of the previous link)
    p_next = HT_next(1:3,4); % body reference frame, z direction is the axis of the link cylinder, affected by the rotation of the ith joint

    % Construct the common fields of link
    link = struct('idx', idx,... % index of this link in the finger
            'radius', crdCyl.rho,... % radius of the link % 'base', HT_this,... % [deprecated]
            'HT_this', HT_this,...
            'HT_next', HT_next,... % homogeneous transform matrix from finger base to link base
            'lactv', lactv,... % link active. If this link is active, the contact point on this link is enabled
            'symbolic', []); % symbolic expression of link points
    
    R = HT_next(1:3,1:3); % NOTICE THAT USE THE NEXT_LINK AS ORIENTATION REFERENCE
    link.p = p_this;
    link.r = R(:,1); % X direction is the radial direction
    link.n = R(:,2); % Y direction is the normal direction (pointing palm)
    link.L = norm(p_next-p_this); % constant
    
    %%% Following are for checking the results
    if ~exist('epsilon','var')
        param = load('problem_config.mat','epsilon');
        epsilon = param.epsilon; % numerical round-off error tolerance
    end
    
    if norm(dot(link.r, link.n))>epsilon % inner product is not close to zero, two vecters are not perpendicular % Notice that cannot use 0 in judgement here.
        error(['Vectors are not perpendicular. Link idx: ', num2str(idx), ' dot product: ', num2str(dot(link.r, link.n))]);
    end
    
    if link.L > epsilon % non-zero link
        link.is_real = true;
        link.contact = calcContactPoint(link, crdCyl); % struct of contact point
        link.geo_mesh = mySGplotLink(link.HT_this(1:3,4),link.HT_next(1:3,4),link.radius);
    else
        link.is_real = false;
        link.contact = []; % empty, indicates that contact here is not possible
        link.geo_mesh = []; % empty, indicates that geo_mesh here is not possible
    end
end