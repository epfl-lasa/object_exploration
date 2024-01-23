function [obj, obj_grad, ht_obj, ht_obj_grad] = symObjectiveFunction(hand, param)
% Reference: Predicting precision grip grasp locations on three-dimensional objects, Lina K. Klein et al. 2020

    fprintf('\nConstructing Objective Function: \n');

    os_info = param.os.os_info;
    
    X_key = param.X_key;
    ncp = param.ncp;
    
    if ncp > 2
        error('NotImplementedError: objective function only work for 2-contact case.');
    end

    Cp = sym(zeros(ncp,3)); % symbolic expression of contact points
    Cn = sym(zeros(ncp,3)); % symbolic expression of contact normal
    
    for i = 1:ncp
        if ismember(0,os_info{i}) % contact on palm
            Cp(i,:) = hand.P.contact.symbolic.p; % contact point on palm
            Cn(i,:) = hand.P.contact.symbolic.n;
            
            Kd = Kd + 1; % palm active
        else
            [idx_f,idx_l] = deal(os_info{i}(1),os_info{i}(2));
            finger = hand.F{idx_f};
            link = finger.Link{idx_l};

            idx_cp = numel(finger.Link)*(idx_f-1)+idx_l; % index of the contact point in the hand symbolic expression of contact points, Notice that the extra link for fingertip
            Cp(i,:) = subs(hand.symbolic.Cp(:,idx_cp),...
                {['rho',num2str(idx_f),num2str(idx_l)],'L'},... % subs constants: link radius and link length
                {link.radius, link.L}); % contains symvars: [ phi, qx, qx+1, qx+2, qx+3, rho, alp]
            
            Cn(i,:) = subs(hand.symbolic.Cn(:,idx_cp),...
                {['rho',num2str(idx_f),num2str(idx_l)],'L'},...
                {link.radius, link.L});
        end
    end

    %%% Cost 1: Force closure cost
    obj_cntr = [sym('x'),sym('y'),sym('z')]; % (1,3)
    cpt = Cp(1,:); % (1,3) contact point thumb
    ft = obj_cntr - cpt; % (1,3) surface normal at contact point. t: thumb, f direction, pointing towards object center
    cpi = Cp(2,:); % (1,3) contact point index
    fi = obj_cntr - cpi; % i: index finger
    
    Cfc = atan2(norm(cross(ft,(cpi-cpt))), dot(ft,(cpi-cpt)))+...
        atan2(norm(cross(fi,(cpt-cpi))), dot(fi,(cpt-cpi))); % 'Cost force closure'
    
    %%% Cost 2: Torque cost
    fg = sym([0,0,-1]); % (1,3) (direction of) force of gravity
    Ct = norm(cross((obj_cntr-cpi),-fg) + cross((obj_cntr-cpt),-fg));

    %%% minimize joint angle movement
    Q = X_key(param.idx_q);
    W = eye(numel(Q)); % cost weight matrix
    Cq = (Q(:).')*W*Q(:); % sum of square

    %%% objective function
    eta = 0.5; % balance coefficient
    obj = eta*(Cfc + Ct) + (1-eta)*Cq;
    
    %%% Save symbolic form of objective function
    matlabFunction(obj,'File','../database/symbolic_functions/objfun','Vars',X_key,'Optimize',false);
    
    %%% Calculate gradient and save
    obj_grad = transpose(jacobian(obj,X_key));
    matlabFunction(obj_grad,'File','../database/symbolic_functions/objfun_grad','Vars',X_key,'Optimize',false);
    
    disp('  Completed.');

    if nargout > 2
        ht_obj = matlabFunction(obj,'Vars',X_key);
        ht_obj_grad = matlabFunction(obj_grad,'Vars',X_key);
    end
end