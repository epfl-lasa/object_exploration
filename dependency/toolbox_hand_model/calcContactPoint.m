function contact = calcContactPoint(link, crdCyl)
    % Calculate the contact point 'p' on the surface of the finger link (modeled as a cylinder).
    % 
    % Input:
    % - link: reference point used to calculate the contact in the link. Use base of the link by default.
    %   Reference point can be obtained from 'makeLink' function.    
    %    - link.p: (3,1), position, reference point of the link;
    %    - link.r: (3,1), radial direction of the link (pointing from the link base to the next base)
    %    - link.n: (3,1), normal direction of the link (pointing from the ctr towards the inner (front) surface of the finger)
    %    - link.L: length of the link, used as upper boundary of z value. This is half of the link length.
    % 
    % - crdCyl: cylindrical coordinates of contact point
    %    - crdCyl.rho: distance, radius of the cylinder;
    %    - crdCyl.phi*: angle (in radius, [-pi, pi]), rotation angle of the normal vector (located at the center point, pointing to front)
    %    - crdCyl.alp*: [0,+1]: translation of the contact point on the surface of cylinder. alp=0, the base; alp=1, the next base or finger tip;
    %   Notice that the definition of alp is the same as in mySGaddContact.
    % 
    % - k: used to construct the approximation of friction cone. k is the
    % number of edges.
    % 
    % - f_mu: friction coefficient
    % 
    % Output:
    % - contact: struct, desired contact point
    %    - contact.p: position (Cartesian coordinate)
    %    - contact.r: radial direction at this contact point (should point along the link, same as link.r)
    %    - contact.n: normal direction, pointing to the outside of the surface (transformed from link.n)
    %    - contact.crdCyl = [rho, phi, alp]: cylindrical coordinates of the contact
    %       point (use alp, the ratio, instead of z, the absolute value
    %    - contact.crdCrt = [x,y,z], Cartesian coordinates of the contact point in the world coordinate frame
    %    - contact.cactv: if this contact is active or not
    %    - contact.HTr2c: homogeneous transformation mtx from link reference point to contact point
    % 
    % Notice: parameters marked with '*' can be used as variables in optimization.
    
    if nargin<2 % for update, just use the current default value
        if isfield(link, 'contact')
            if isfield(link.contact, 'crdCyl')
                crdCyl = link.contact.crdCyl;
            else
                error('[calcContactPoint]: crdCyl does not exist in "contact".');
            end
        else
            error('[calcContactPoint]: crdCyl does not exist in "link".');
        end
    end
    
    if isfield(link, 'contact') % already exists, contains symbolic form of contact. just copy the field.
        contact = link.contact;
    end

    contact.cactv = link.lactv; % same as the information of the link
    
    rho = crdCyl.rho;
    phi = crdCyl.phi;
    
    alp = crdCyl.alp;
    z = alp*link.L; % scaler, z coordinate, calculated as a ratio of link length, range [-L, +L]
    
    %%% Introducing refCF: this is the reference frame of contact point. It
    %%% locates at the base (head, reference point) of the link, and has
    %%% the same orientation as link.HT_next (at the end of the link). It
    %%% should be fixed w.r.t. the link.
    
    refCF = link.HT_next;
    tr = [1,0,0]*(-link.L);
    localTransf_test = trvec2tform(tr);
    contact.refCF = refCF*localTransf_test; % reference CF USED TO CALCULATE the contact point (see above explanation)
    
    % POST-MULTIPLICATION if transform w.r.t. current reference frame
    %%% Step 1: rotate w.r.t. the CURRENT radial direction (x direction)
    rotm = rotx(rad2deg(phi)); % [NOTICE] rotx is in DEGREE
    ctCF = contact.refCF * rotm2tform(rotm);
    
    %%% Step 2: translate in normal direction (y of contact CF)
    tn = [0,1,0]*rho; % normal direction: y direction
    ctCF = ctCF * trvec2tform(tn);
    
    %%% Step 3: translate in radial direction (x of contact CF)
    tr = [1,0,0]*z;
    ctCF = ctCF * trvec2tform(tr); % contact CF of the contact point
    
    contact.r = ctCF(1:3,1); % radial (x of local CF) direction
    contact.n = ctCF(1:3,2); % normal (y of local CF) direction
    contact.p = ctCF(1:3,4); % contact position, expressed in WCF
    
    %%% Step 4: from link Reference (i.e. link base) to Contact homogeneous transformation
    %% Alternative way to obtain the ctCF
    contact.HTr2c = rotm2tform(rotm)*trvec2tform(tn)*trvec2tform(tr); % Local, from the reference point (base) of the link (to which the contact belongs), to the contact point on the link
    contact.HTcp = contact.refCF*contact.HTr2c; % GlobalrefCF, in the kinematic chain, from the base to the contact point, expressed in the WCF
    %%% Notice that: (1) ctCF == contact.HTcp, (2) contact.p == contact.crdCrt
    
    contact.crdCrt = contact.HTcp(1:3,4); % Cartesian coordinates of contact points, in world CF
    
    contact.crdCyl.rho = rho; % cylindrical coordinates of the contact
    contact.crdCyl.phi = phi;
    contact.crdCyl.alp = alp;
    
    param = load('problem_config.mat','S');
    S = param.S;
    contact.FC = contact.HTcp(1:3,1:3)*S; % approximated edges of friction cone. HTcp transforms points (S) in {W} to {C} (contact CF)
end