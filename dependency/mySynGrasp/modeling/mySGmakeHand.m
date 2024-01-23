function hand = mySGmakeHand(F,T)

	% T: homogeneous transformation mtx of hand base coordinate frame
    if nargin < 2
        T = eye(4);
    end
    
	%%% Pre-allocation
	hand = struct('F',[],... % cell array of fingers
        'T',[],... % homogeneous transformation matrix
		'n',[],... % number of fingers
		'm',[],... % total number of dofs
		'q',[],... % (m,1), list of all joints
		'qin',[],... % (m,1), index, to which finger does the corresponding joint angle belong
		'qinf',[],... % (m,1), indexing of the joint in the finger
        'limit',[],... % (m,2), hand joint (:,1) lower limit and (:,2) upper limit 
        'qactv',[],... % active index of the joint anlge q. 1: active, 0: not.
		'ctype',[],... % contact type, [1,2,3]
		'cp',[],... % (7,ncp), contacts on the hand
	    'ftips',[],... % (3,n) finger tips positions
	    'S',[],... % synergy matrix, assigned to the hand structure with function `mySGdefineSynergies`
	    'Kq',[],... % (m,m), joint stiffness matrix, symmetric and positive definite
	    'Kz',[],... % (nz,nz), synergy stiffness, symmetric and positive definite matrix
	    'Jtilde',[],... % (ncp,m), the complete hand Jacobian matrix
	    'H',[],... % the selection matrix
	    'J',[],... % `hand.J = hand.H * hand.Jtilde`, the Hand Jacobian
	    'JS',[]); % (nl,nz), `hand.JS = hand.J * hand.S`, the underactuated Jacobian

	hand.T = T;
	hand.F = F;
	hand.n = length(F); % number of fingers

	m = 0; % total number of joints
	for i = 1:hand.n
	    if(~SGisFinger(F{i}))
	        error('Argument F contains a non-finger struct @ %d', i)
	    end
	    m = m + F{i}.n; % add number of joints (F{i}.n is the number of joints of ith finger)
	end

	hand.m = m; % number of degrees of freedom (joints) of the hand
	for i = 1:hand.n % iterate over fingers
	    hand.q = [hand.q; F{i}.q]; % hand.q is an m*1 vector, each F{i}.q is n*1 vector
	    hand.qin = [hand.qin; i*ones(size(F{i}.q))]; % 'q' index, (m,1), a list of identical finger idx, to indicate to which finger does this q belong, e.g. [1,1,1,1,2,2,2,2,...]^T	    
	    hand.qinf = [hand.qinf; F{i}.qin]; % (m,1), 'q' index of finger. indicate which order is joint q in the finger. e.g. [1,2,3,4,1,2,3,4,...]^T
		hand.qactv = [hand.qactv; F{i}.active_joints]; % (m,1), list of flags for active joints.
        hand.limit = [hand.limit; [F{i}.lb(:),F{i}.ub(:)]]; % (:,1): lower limit, (:,2): upper limit
	end

	hand.ctype = 1;

	hand.ftips = mySGfingertips(hand); % (3,n), all fingertip positions, should be equal to a matrix of hand.F{i}.joints_pos
	hand.S = eye(size(hand.q,1),size(hand.q,1));

	hand.Kq = eye(m);
	hand.Kz = eye(m);
	hand.Kw = eye(6);

	hand.factv = ones(hand.n,1); % set activity of fingers
    
	hand = mySGjoints(hand); % update Cartesian positions of joints by retrieving the FK of each finger

	hand.VF = {}; % initialize the cell array of virtual fingers
	hand.nvf = 0; % number of virtual fingers
    
    %% Symbolic variables of hand model
    %%% integrate symbolic form of contact points to the hand
    Cp = []; % (3, Ncp) to save the Ncp in total contact points
    Cn = []; % contact normal
    Cr = []; % radial direction of contact
    for i = 1:hand.n
        finger = hand.F{i};
        if ~isempty(finger.symbolic)
            try
                for j = 1:finger.n+1 % number of links on finger (plus fingertip)
                    Cp = cat(2, Cp, finger.Link{j}.contact.symbolic.p); % contact point coordinates
                    Cn = cat(2, Cn, finger.Link{j}.contact.symbolic.n); % contact normal directions
                    Cr = cat(2, Cr, finger.Link{j}.contact.symbolic.r); % radial directions of link (from base to next base)
                end
            catch
                disp(finger.idx);
            end
        else
            warning('Finger symbolic form does not exist.');
        end
    end
    hand.symbolic.Cp = Cp;
    hand.symbolic.Cn = Cn;
    hand.symbolic.Cr = Cr;
end