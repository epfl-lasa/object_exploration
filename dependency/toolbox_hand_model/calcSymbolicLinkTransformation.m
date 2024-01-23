function finger = calcSymbolicLinkTransformation(finger)
    %%% Obtain symbolic expression from base to each contact points on the link
    % Adapted from 'moveFinger.m'

    if isfield(finger, 'symbolic')
        if ~isempty(finger.symbolic)
            return;
        end
    else
        error 'Symbolic field does not exist.';
    end

    param = load('problem_config.mat');
    database_dir = param.database_dir;
    k = param.k;
    f_gamma = param.f_gamma;
    S = sym(param.S);

    syms L x y z; % Initialize symbolic variables
    q = finger.q_sym;
    Q = sym(['q',num2str(finger.idx),'%d'],[1 finger.n]);

    symbolic.base = sym(finger.base);
    symbolic.DHpars = sym(finger.DHpars); % symbolic form
    symbolic.DHpars(:,3) = [q(1), q(2), q(3), q(4)]; % create a list of symbolic variables for joint angles
    
    
    %%% [Allegro Hand DH Offset in JOINT ANGLE VARIABLES]
    if strcmp(finger.hand_type, "AllegroHandLeft")
        dh_offset_allegro_left = sym([0, 0, 0, 0;...
            pi/2, pi/2, pi/2, pi/2;...
            0, 0, 0, -pi/2;...
            0, 0, 0, 0]);
        dh_offset_allegro_left = dh_offset_allegro_left(:,[4,1,2,3]);
        symbolic.DHpars(:,3) = symbolic.DHpars(:,3) + dh_offset_allegro_left(:,finger.idx);
    end
    if strcmp(finger.hand_type, "AllegroHandRight")
        dh_offset_allegro_right = sym([0, 0, 0, 0;...
            pi/2, pi/2, pi/2, pi/2;...
            0, 0, 0, pi/2;...
            0, 0, 0, 0]);
        dh_offset_allegro_right = dh_offset_allegro_right(:,[4,1,2,3]);
        symbolic.DHpars(:,3) = symbolic.DHpars(:,3) + dh_offset_allegro_right(:,finger.idx);
    end
    
    
    symbolic.joints_pos = sym(zeros(3,finger.n+1)); % (3,nl+1) save symbolic expression of finger joints Cartesian positions
    referenceJoint = symbolic.base; % should be a [4,4] matrix

    for l = 1:finger.n+1 % (n+1)th link is fingertip
        alp = sym(['alp',num2str(finger.idx),num2str(l)]); % alpij, i: finger index, j: link index
        phi = sym(['phi',num2str(finger.idx),num2str(l)]);
        rho = sym(['rho',num2str(finger.idx),num2str(l)]);

        %%% Calculating the symbolic transformation from base of the link to the contact point of the link based on the cylindrical coordinate, see 'calcContactPoint.m'
        rotm = sym([1,0,0;...
            0,cos(phi),-sin(phi);... % phi is in degree
            0,sin(phi), cos(phi)]);
        tn = [0;rho;0]; % translate to the surface of link
        tr = [L*alp;0;0]; % move along z axis. separate the symbolic of L, the link length, and alp: alpha

        HTr2cp = [rotm,[0;0;0];0,0,0,1]*[eye(3),tn;0,0,0,1]*[eye(3),tr;0,0,0,1]; % from reference point (base of link) to contact point
        HTr2lc = [rotm,[0;0;0];0,0,0,1]*[eye(3),tr;0,0,0,1]; % HT reference point to link center

        if l < finger.n+1
            refJoint_old = referenceJoint; % the old ref on the base
            localTransf = mySGDHMatrix(symbolic.DHpars(l,:), true); % set the 2nd parameter as 'true' to obtain the symbolic form
            referenceJoint = referenceJoint * localTransf; % the new ref locates on the tip of the link
            symbolic.joints_pos(:,l) = referenceJoint(1:3,4); % Cartesian position of l^{th} joint
        else % for the (n+1)th link: the fingertip virtual link
            refJoint_old = referenceJoint; % referenceJoint is not updated
        end

        localTransf_test = sym([1,0,0,-L;...
            0,1,0,0;...
            0,0,1,0;...
            0,0,0,1]);
        refCF = referenceJoint * localTransf_test; % refer to 'calcContactPoint'

        HTcp = refCF * HTr2cp; % (contact point), set base as the reference point of the link % contact point: HTcp(1:3,4)
        HTlc = refCF * HTr2lc; % (link center), at the same height as the contact point in local frame, but at the central axis of link

        finger.Link{l}.contact.symbolic.refCF = refCF; % reference CF of contact point: locats at the base of the link

        finger.Link{l}.symbolic.HT_this = refJoint_old; % (4,4)
        finger.Link{l}.symbolic.HT_next = referenceJoint; % (4,4)

        matlabFunction(refJoint_old,...
            'File',[database_dir, '/symbolic_functions/','F',num2str(finger.idx),'L',num2str(l),'_HT_this'],'Vars',Q);
        matlabFunction(referenceJoint,...
            'File',[database_dir, '/symbolic_functions/','F',num2str(finger.idx),'L',num2str(l),'_HT_next'],'Vars',Q);

        finger.Link{l}.contact.symbolic.HTr2cp = HTr2cp;
        finger.Link{l}.contact.symbolic.HTr2lc = HTr2lc;

        finger.Link{l}.contact.symbolic.HTcp = HTcp; % contact point, from finger base to the contact point
        finger.Link{l}.contact.symbolic.HTlc = HTlc; % from finger base to link center

        % Notice that this is contact p r n, different from link p r n
        finger.Link{l}.contact.symbolic.r = HTcp(1:3,1); % radial (x of local CF) direction on the contact
        finger.Link{l}.contact.symbolic.n = HTcp(1:3,2); % normal (y of local CF) direction
        finger.Link{l}.contact.symbolic.p = HTcp(1:3,4); % contact position, expressed in WCF, this is used to construct 'hand.symbolic.Cp'

        finger.Link{l}.contact.symbolic.lc = HTlc(1:3,4); % Cartesian coordinates of link center

        %%% [link dist] dist from link to an external point
        % The distance from the link to a 3D position (x,y,z) in the space,
        % used in the nonlinear constraint for collision avoidance
        x0 = [x;y;z]; % space point (e.g. object center)
        x1 = refJoint_old(1:3,4); % reference point of link_this
        x2 = referenceJoint(1:3,4); % reference point of link_next
        
        try
            if finger.Link{l}.is_real % for real link
                link_dist = norm(cross(x0-x1,x0-x2))/norm(x2-x1);
            else
                link_dist = norm(x0-x1); % for virtual links
            end
        catch
            disp(finger.idx);
        end
        finger.Link{l}.symbolic.link_dist = link_dist; % [q1...4, x, y, z]

        %%% [contact point dist] dist from contact point to an external point
        d = finger.Link{l}.contact.symbolic.p(:) - x0; % from an external point (e.g. object CoM), pointing towards contact position on the link
        finger.Link{l}.contact.symbolic.d = d;
        cp_dist = d'*d; % Eucledian distance from contact point to a space point
        finger.Link{l}.contact.symbolic.cp_dist = cp_dist; % [ L, alp, phi, q1...4, rho, x, y, z]

        %%% [approximation of force cone]
        % [See: test_cone_approximation.m]
        R = HTcp(1:3,1:3);
        FC = R * S; % approximation of friction cone with edges [Left multiply: w.r.t. world CF]
        % FC = FC./sqrt(ones(1,3)*(FC.*FC)); % (3,k=6) Normalization of force closure edges
        finger.Link{l}.contact.symbolic.FC = FC; % [ L, alp, phi, q1...4, rho]

        %%% [approximation of torque cone]
        r_mtx = sym(repmat(d,1,k)); % (3,k=6), matrix of 'r' in torque calculation
        TC = cross(r_mtx,FC);
        finger.Link{l}.contact.symbolic.TC = TC; % [ L, alp, phi, q1...4, rho, x, y, z]

        vn_i = -d; % (3,1) % from contact point, pointing towards object center
        vn_i = vn_i./sqrt(ones(1,3)*(vn_i.*vn_i)); % vectorized form of normalization, equivalent to: vn_i = vn_i./norm(vn_i);
        TC_torsinal = f_gamma*vn_i; % Torsinal torque (soft finger)
        finger.Link{l}.contact.symbolic.TC_torsinal = TC_torsinal; % [ L, alp, phi, q1...4, rho, x, y, z]
    end

    symbolic.tip = referenceJoint; % this is the fingertip
    symbolic.joints_pos(:,end) = referenceJoint(1:3,4);

    matlabFunction(symbolic.tip,...
        'File',[database_dir, '/symbolic_functions/','F',num2str(finger.idx),'_tip'],'Vars',Q);
    matlabFunction(symbolic.joints_pos,...
        'File',[database_dir, '/symbolic_functions/','F',num2str(finger.idx),'_joints_pos'],'Vars',Q);

    finger.symbolic = symbolic;
    % fprintf('Symbolic expression constructed for finger: %d\n', finger.idx);
end