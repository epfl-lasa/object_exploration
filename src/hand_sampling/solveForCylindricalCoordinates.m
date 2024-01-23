function [alp, phi, rho] = solveForCylindricalCoordinates(hand, iF, realContacts, X_key, params, real_q, real_posH, real_quatH)
    % realContacts: (3,4), contact point, represented in World reference frame
    % Check `calcContactPoint` for the calculation of contact points from
    % cylindrical coordinates
    
    
    cLink = hand.F{iF}.Link{end-1}; % link in contact, last real link
    realContact = realContacts(:,iF); % (3,1)
    
    % linkHT = cLink.symbolic.HT_next; % (4,4), reference HT is the next HT
    linkHT = cLink.symbolic.HT_this;
    
    linkHTCtr = linkHT(1:3,4); % [ qi1, qi2, qi3, qi4, qH1, qH2, qH3, qH4, xH, yH, zH]
    linkHTCtr_num = double(subs(linkHTCtr,...
        [X_key(params.idx_q), X_key(params.idx_posH), X_key(params.idx_quatH)],...
        [real_q, real_posH, real_quatH])); % (3,1)
    
    pLinkCF = realContact - linkHTCtr_num; % (3,1), contact point represented in link coordinate frame
    
    pX = pLinkCF(1); % rho * cos(phi)
    pY = pLinkCF(2); % rho * sin(phi)
    pZ = pLinkCF(3);
    
    alp = pZ/cLink.L;
    rho = cLink.radius;
    phi = atan(pY./pX); % in rad
end