%%% This function visualizes the optimization results, including: hand
%%% model, object model, contact points
function visualizeOptimizationConfig(hand, object_list, X_sol, param, fig_title, if_save)
% hand: hand model
% object: object model
% X_sol: solution obtained in the optimization process

    if nargin < 6
        if_save = false;
    end

    X_key = param.X_key;
    os_info = param.os.os_info;
    k = param.k;
    ncp = param.ncp;
    qactv = param.qactv_loop;
    
    hand.q(qactv) = X_sol(param.idx_q); % update hand joint angles
    hand = mySGmoveHand(hand);
    
    %%%%%%%%%%
    figure, hold on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    view([-150 30]);
    
    if nargin>4
        title(fig_title);
    end
    
    axis equal;
    grid on;
    
    %%% Visualize the hand model
    mySGplotHand(hand); hold on;
    
    %%% Visualize the object model
    plot_contact_info = false;
    
    oc = X_sol(param.idx_oc); % object center
    if ~isempty(object_list) % no objects is given, do not plot
        switch class(object_list)
            case 'struct' % a single object is given
                soln = struct('X_sol',X_sol,'param',param);
                object = updateObjectConfig(object_list, soln);
                mySGplotObject(object);
                hold on;
                scatter3(oc(1),oc(2),oc(3),100,'b','filled');
                plot_contact_info = true;
            case 'cell' % multiple objects available
                for i = 1:numel(object_list)
                    obj_i = object_list{i};
                    if ~isfield(obj_i,'clr')
                        warning('Object color not specified.');
                    end
                    if (~isempty(obj_i)) && isa(obj_i,'struct')
                        oc_i = obj_i.center;
                        mySGplotObject(obj_i); 
                        hold on;
                        scatter3(oc_i(1),oc_i(2),oc_i(3),100,'b','filled');
                        hold on;
                    else
                        fprintf('Nr. %d object in the given grasped objects list is empty.\n', i);
                    end
                end
            case 'char' % input: object = 'default'
                % object is not given, but still plot, create one from given information
                object_list = mySGsphere(trvec2tform(oc(:).'), param.object_radius);
                mySGplotObject(object_list);
                scatter3(oc(1),oc(2),oc(3),100,'b','filled');
        end
    end
    
    %%% Visualize the contact points and normals [ONLY FOR SINGLE OBJECT]
    if plot_contact_info
        Cp = hand.symbolic.Cp;
        Cn = hand.symbolic.Cn;

        pvec = X_sol(param.idx_pvec);
        pc = hand.P.contact.symbolic.p; % pc stands for 'palm contact'
        pc = double(subs(pc, symvar(pc), pvec(:).'));
    
        W = zeros(6,ncp*k); % (6,ncp*k) to save the F-cone and T-cone of all contact points
        for i = 1:ncp
            if ~all(os_info{i}) % include 0, this is palm
                cp = pc; % assgin palm contact to contact point
                cn = sym(hand.P.contact.symbolic.n);
                contact = hand.P.contact;

                key_cn = ismember(X_key, symvar(cn));
                cn = double(subs(cn, X_key(key_cn), X_sol(key_cn)));

                if param.cstr.fc
                    FC_i = contact.symbolic.FC;
                    key = ismember(X_key, symvar(FC_i));
                    FC_i = double(subs(FC_i, X_key(key), X_sol(key)));
                    scatter3(cp(1),cp(2),cp(3),100,'b','filled');
                    quiver3(cp(1),cp(2),cp(3),cn(1),cn(2),cn(3)); % contact normal direction
                    plotVectorCone(cp,FC_i,'r',10);
                    hold on;

                    TC_i = contact.symbolic.TC; % [ L, alp, phi, q1, q2, q3, q4, rho, x, y, z]
                    key = ismember(X_key, symvar(TC_i));
                    TC_i = double(subs(TC_i, X_key(key), X_sol(key)));

                    % plotVectorCone(cp,TC_i,'g',10);
                    % hold on;
                    W_i = cat(1,FC_i,TC_i);
                    W(:,(i-1)*k+1:i*k) = W_i;
                end
            else
                [idx_f,idx_l] = deal(os_info{i}(1),os_info{i}(2));
                finger = hand.F{idx_f};
                link = finger.Link{idx_l};

                idx = numel(finger.Link)*(idx_f-1)+idx_l; % notice that the numel of Link includes fingertip, 1 more than finger.n
                rho_i = ['rho',num2str(idx_f),num2str(idx_l)]; % name of the rho variable specified to the link

                cp = Cp(:,idx); % [ L, alp, phi, q1, q2, q3, q4, rho]
                cp = subs(cp, {rho_i,'L'}, {link.radius,link.L});
                key_cp = ismember(X_key, symvar(cp));
                cp = double(subs(cp, X_key(key_cp), X_sol(key_cp)));

                cn = Cn(:,idx); % [ phi14, q11, q12, q13, q14]

                key_cn = ismember(X_key, symvar(cn));
                cn = double(subs(cn, X_key(key_cn), X_sol(key_cn)));

                contact = link.contact;

                if param.cstr.fc % If force closure is a constraint, plot force closure
                    FC_i = contact.symbolic.FC; % [ L, alp, phi, q1, q2, q3, q4, rho]
                    FC_i = subs(FC_i, {rho_i,'L'}, {link.radius,link.L});
                    key = ismember(X_key, symvar(FC_i));
                    FC_i = double(subs(FC_i, X_key(key), X_sol(key)));

                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    scatter3(cp(1),cp(2),cp(3),100,'b','filled');
                    quiver3(cp(1),cp(2),cp(3),cn(1),cn(2),cn(3)); % contact normal direction
                    plotVectorCone(cp,FC_i,'r',10);
                    hold on;
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    TC_i = contact.symbolic.TC; % [ L, alp, phi, q1, q2, q3, q4, rho, x, y, z]
                    TC_i = subs(TC_i, {rho_i,'L'}, {link.radius,link.L});
                    key = ismember(X_key, symvar(TC_i));
                    TC_i = double(subs(TC_i, X_key(key), X_sol(key)));

                    % plotVectorCone(cp,TC_i,'g',10);
                    % hold on;

                    % Notice that torsinal torque can has different directions
                    %{
                    TC_t_i = contact.symbolic.TC_torsinal; % torsinal torque component
                    TC_t_i = subs(TC_t_i, {rho_i,'L'}, {link.radius,link.L});
                    key = ismember(X_key, symvar(TC_t_i));
                    TC_t_i = double(subs(TC_t_i, X_key(key), X_sol(key)));
                    %}

                    %%% Visualization of FC and TC
                    W_i = cat(1,FC_i,TC_i);
                    W(:,(i-1)*k+1:i*k) = W_i;
                end
            end
            scatter3(cp(1),cp(2),cp(3),100,'g','filled');
            quiver3(cp(1),cp(2),cp(3),cn(1),cn(2),cn(3));
        end
    end
    
    if if_save
        savefig([fig_title,'.fig']);
    end
    
    %{
    disp('Evaluation of force closure property.');
    %%% Evaluate the FC property by searching for non-trivial solution for
    %%% W*x = 0. For alternative methods, see 'forceClosureCheck.m'.
    
    if param.cstr.fc
        [U,S,V] = svd(W);
        x = V(:,end);
        if sum(W*x)<1e-6
            disp('Force closure exists.');
        else
            disp('Force closure does not exist.');
        end
    end
    %}
    
end