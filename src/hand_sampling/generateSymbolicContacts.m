function [sym_contacts] = generateSymbolicContacts(hand,to_save)
    % See: 'calcSymbolicLinkTransformation.m' for definition of reference frames

    if nargin < 2
        to_save = true;
    end

    positions = sym(zeros(3,4)); % contact points of all 4 fingers
    normals = sym(zeros(3,4)); % contact normal of all 4 fingers
    
    for iF = 1:hand.n
        finger = hand.F{iF};
        distal = finger.Link{end-1}; % Link{end} is fingertip (virtual frame)
        assert(distal.L > 0);
        HTcp = distal.contact.symbolic.HTcp;
        HTcp = subs(HTcp, {'L', ['rho',num2str(iF),'4']}, [distal.L, distal.contact.crdCyl.rho]);
        HTcp = hand.sym_T * HTcp; % Consider palm transformation
        
        positions(:,iF) = HTcp(1:3,4);
        normals(:,iF) = HTcp(1:3,2);
    end
    
    sym_contacts.positions = positions;
    sym_contacts.normals = normals;
    
    % assert(all(ismember(symvar(hand.sym_T),symvar(positions))));
    % assert(all(ismember(symvar(hand.sym_T),symvar(normals))));
    
    if to_save
        global path_hand_model % Need to be saved together with hand model
        save(fullfile(path_hand_model,'sym_contacts.mat'), 'sym_contacts');
    end
end