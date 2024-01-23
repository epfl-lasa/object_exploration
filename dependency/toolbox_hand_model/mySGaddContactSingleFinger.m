function [finger, contact] = mySGaddContactSingleFinger(finger, link_idx, crdCyl, type)
    % [CUSTOMIZED] Add contact on the desired link (default fingertip) of
    % ONE SINGLE finger. This function is used before the
    % finger jacobian can be calculated (in fact, a block of the hand
    % jacobian), because finger Jacobian starts from the base and ends at
    % the contact point. See: calcJacobianSingleFinger
    
    % Input Arguments:
    %   * finger = the finger structure on which the user wants to place the contact point
    %   * type = a flag indicating the contact type with the following semantics:
    %       1: hard finger (2D or 3D);
    %       2: soft finger (2D or 3D);
    %       3: complete constraint (2D or 3D);
    %   * % cwhere (omitted) = the fingers on which contact points will be placed (5 by default)
    %   * link_idx: the link where the contact point lies
    %   * crdCyl:
    %       * crdCyl.rho: radius of cylinder
    %       * crdCyl.phi: angles, [-pi, pi]
    %       * crdCyl.alp: alpha, [0,1], an argument that parameterize the position of the contact point on the link (0 = link base; 1 = link end)
    
    if nargin < 4
        type = 1; % hard finger
    end
    if nargin < 3
        if nargin < 2
            link_idx = finger.n; % by default, add contact to fingertip
        end
        crdCyl = struct('rho', finger.Link{link_idx}.radius,... % radius of the link
            'phi', 0,... % do not rotate (add the contact at the front on the surface of the link)
            'alp', 0.5); % middle of the link (add the contact in the middle of the surface of the link)
    end
    if link_idx > finger.nlink
        error('Link number exceeds limit.');
    end

    %%%% Get the reference point of the contact link
    %%% [old version]
    %{
    referenceJoint = finger.base;
    for i=1:link_idx-1
        localTransf = mySGDHMatrix(finger.DHpars(i,:));       
        referenceJoint = referenceJoint*localTransf;
    end
    
    % Calculate the transform to the contact point on the last link
        % tmp = finger.DHpars(link,:);
        % tmp(2) = tmp(2) * alp; % tmp(2): 'a' in DH table. alpha=0, link base; alpha=1, link tip
        
        % localTransf = mySGDHMatrix(tmp);
        % referenceJoint = referenceJoint*localTransf; % not necessarily the end effector

    %%% Updated: Calc HT of arbitrary contact point
    HT_this = referenceJoint;
    HT_next = referenceJoint*mySGDHMatrix(finger.DHpars(link_idx,:));
    tmp_link = makeLink(HT_this, HT_next);
    %}
    %%% [updated version]
    %%%% Calculate the HT from finger base to contact point
    contact = calcContactPoint(finger.Link{link_idx}, crdCyl); % tmp contact point
    
    %%% [old version]
    % localTransf = contact.HTr2c; % Transf from reference point ('tmp_link_ref') to contact point ('tmp_cp')
    % referenceJoint = referenceJoint*localTransf;
    
    %%% [updated version] use link base
    % referenceJoint = link.base*contact.HTr2c; % consider the contact point as one 'referenceJoint'
    % [update 2]: use contact.crdCrt as the cartesian coordinate of the contact

    ncp = size(finger.cp,2); % finger.cp: (7, ncp), number of current contact        
    contact = zeros(7,1); % column vector, description of the new contact to be added
    % contact(1:3) = referenceJoint(1:3,4); % cartesian position
    contact(1:3) = contact.crdCrt;
    contact(4) = 0; % 'cwhere', set as 0, since the cwhere is used for hand to indicate which finger is in contact. meaningless for finger.
    contact(5) = link_idx; % link number where the contact point is (by default the last one)
    contact(6) = crdCyl.alp; % [deprecated: for built in function] a scalar value that parameterize the position of the contact point on the specified link
    contact(7) = type; % the contact type
    
    % check for existing contact points on the same position. if not, add it.
    if ncp == 0
        finger.cp = contact;
    elseif ~ismember(contact', finger.cp', 'rows')
        finger.cp(:,ncp+1) = contact; % (7, ncp)
    end

    finger.Link{link_idx}.contact = contact; % update the contact on the link
    
    % finger.Jtilde = calcJacobianSingleFinger(finger); % reference: mySGjacobianMatrix
end