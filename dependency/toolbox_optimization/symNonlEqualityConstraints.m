% Obtain the symbolic form of nonlinear inequality constraints
function [ceq, ceq_grad, param, ht_ceq, ht_ceq_grad] = symNonlEqualityConstraints(hand, param)
    os_info = param.os.os_info;
    ncp = param.ncp;
    r = param.object_radius; % object radius
    k = param.k; % number of edges of approximated friction cone
    X_key = param.X_key;
    cstr = param.cstr;
    oc = [sym('x');sym('y');sym('z')]; % object center
    
    fprintf('\nConstructing Nonl. Equality Constraints: \n');
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Equality Constraint 1: Contact points on object surface
    % dist. between contact points and object ctr equals object radius
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fprintf('* In-contact constraint: ');
    ceq = sym([]); % to save the symbolic form
    ceq_idx = [];
    ceq_name = {};
    
    for i = 1:ncp
        if ~all(os_info{i}) % This is the palm
            pc = hand.P.contact.symbolic.p; % palm contact
            %%% Notice that cp_dist is squared distance!!!
            cp_dist = norm(oc(:)-pc(:))^2; % palm from object center to palm center
        else
            [idx_f,idx_l] = deal(os_info{i}(1),os_info{i}(2));
        
            finger = hand.F{idx_f};
            link = finger.Link{idx_l};

            L = link.L;
            rho_sym = ['rho',num2str(idx_f),num2str(idx_l)]; % name of rho associated with this contact

            cp_dist = link.contact.symbolic.cp_dist; % distance from contact point to object center
            cp_dist = subs(cp_dist, {'L','r',rho_sym}, {L,r,link.radius});
        end
        ceq(end+1) = cp_dist - r*r; % notice that cp_dist is squared; so: cp_dist == r*r;
    end
    ceq_idx(end+1) = numel(ceq);
    ceq_name{end+1} = 'In-contact constraint';
    fprintf('%d\n', ceq_idx(end));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Equality Constraint 3: Force closure
    % Coefficients times wrench cone edges, sum up to 0(6,1)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if cstr.fc
        fprintf('* Force closure: ');
        W = sym(zeros(6,ncp*k)); % A list to save all wrenches
        c = sym('c%d%d',[ncp,k]); % coefficients of friction cone edges in solving the linear programming problem
        c = reshape(c.',[],1);
        for i = 1:ncp
            if ~all(os_info{i}) % palm
                FC_i = hand.P.contact.symbolic.FC;
                TC_i = hand.P.contact.symbolic.TC;
            else
                [idx_f,idx_l] = deal(os_info{i}(1),os_info{i}(2));
                finger = hand.F{idx_f};
                link = finger.Link{idx_l};

                rho_sym = ['rho',num2str(idx_f),num2str(idx_l)];

                FC_i = link.contact.symbolic.FC; % (3,k)
                FC_i = subs(FC_i, {'L','r',rho_sym}, {link.L,r,link.radius});

                TC_i = link.contact.symbolic.TC; % (3,k)
                TC_i = subs(TC_i, {'L','r',rho_sym}, {link.L,r,link.radius});
            end
            W_i = cat(1,FC_i,TC_i); % (6,k) construct wrench vector
            W(:,(i-1)*k+1:i*k) = W_i; % (6,ncp*k) stack wrenches from this contact point to the entire wrench
        end
        ceq(end+1:end+6) = W*c;
    end
    ceq_idx(end+1) = numel(ceq)-sum(ceq_idx);
    ceq_name{end+1} = 'Force closure';
    fprintf('%d\n', ceq_idx(end));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Save symbolic form of nonlinear constraints
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    param.ceq_idx = ceq_idx;
    param.ceq_name = ceq_name;
    
    matlabFunction(ceq,'File','../database/symbolic_functions/nonl_ceq','Vars',X_key,'Optimize',false);
    
    ceq_grad = transpose(jacobian(ceq, X_key)); % size (lens_var, lens_fun)
    
    matlabFunction(ceq_grad,'File','../database/symbolic_functions/nonl_ceq_grad','Vars',X_key,'Optimize',false);
    
    fprintf('  Total num. of nonl. equality constraints: %d\n', numel(ceq));
    
    if nargout > 3
        ht_ceq = matlabFunction(ceq,'Vars',X_key);
        ht_ceq_grad = matlabFunction(ceq_grad,'Vars',X_key);
    end
end