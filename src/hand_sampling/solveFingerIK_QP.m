% This function calculates the weighted IK of a finger model using Quadratic Programming.
% See: solveFingerIK.m
% 
% Notice that QP does not optimize any kinematic metrics and does not take nonlinear constraints.
% 
% Input:
% - tgt_position: (1,3), desired position of the fingertip
% - tgt_normal: (1,3), desired direction of the fingertip's radial
% direction (normal direction on the contact surface)
% - nIteration: number of iterations to solve QP iteratively
% 
% Output:
% - finger: finger struct with updated fields
% - ikSol: solution struct

function [finger, ikSol] = solveFingerIK_QP(finger, tgt_position, tgt_normal, nIteration)
    % Forward Kinematics of finger tip
    % FK = finger.Link{end}.symbolic.HT_next(1:3,4);
    
    global Scores
    
    % Forward Kinematics: use contact point on finger distal (incl. cylindrical coordinates)
    distal = finger.Link{end-1}; % this is the last finger phalanx
    p_sym = distal.contact.symbolic.p; % contact position on fingertip
    r_sym = distal.contact.symbolic.r; % radial direction of finger distal cylinder (surface normal direction)
    % symvar of contactSym: [L, alp14, phi14, q11, q12, q13, q14, rho14]
    
    p_sym = subs(p_sym, {'L', ['rho',num2str(finger.idx),'4']}, [distal.L, distal.contact.crdCyl.rho]); % [alp14, phi14, q11, q12, q13, q14]
    r_sym = subs(r_sym, {'L', ['rho',num2str(finger.idx),'4']}, [distal.L, distal.contact.crdCyl.rho]); % [q11, q12, q13, q14]
    
    % Optimization Variable (fkVars)
    fkVars_sym = symvar(p_sym); % [alp14, phi14, q11, q12, q13, q14], (1,N)
    
    assert(all(ismember(symvar(r_sym),fkVars_sym)));
    assert(isequal(numel(fkVars_sym),6));
    
    idx_q = ismember(fkVars_sym, finger.q_sym); % index of [q11, q12, q13, q14] in the list of fkVars_sym
    idx_alp = ismember(fkVars_sym, ['alp',num2str(finger.idx),'4']);
    idx_phi = ismember(fkVars_sym, ['phi',num2str(finger.idx),'4']);
    
    % Jacobian of contact point p_sym
    %{
    Jp_sym = [];
    if isfield(distal.contact.symbolic, 'Jp_sym')
        Jp = distal.contact.symbolic.Jp_sym;
        if all(ismember(symvar(Jp), fkVars_sym)) % if J exists, just use J
            Jp_sym = Jp;
        end
    end
    if isempty(Jp_sym)
        Jp_sym = jacobian(p_sym, fkVars_sym); % (3, N)
        finger.Link{end-1}.contact.symbolic.Jp_sym = Jp_sym;
    end
    %}
    Jp_sym = jacobian(p_sym, deltaQ_sym);
    
    % Jacobian of radial direction r_sym
    %{
    Jr_sym = [];
    if isfield(distal.contact.symbolic, 'Jr_sym')
        Jr = distal.contact.symbolic.Jr_sym;
        if all(ismember(symvar(Jr), fkVars_sym)) % if J exists, just use J
            Jr_sym = Jr;
        end
    end
    if isempty(Jr_sym)
        Jr_sym = jacobian(r_sym, fkVars_sym); % (3, N)
        finger.Link{end-1}.contact.symbolic.Jr_sym = Jr_sym;
    end
    %}
    Jr_sym = jacobian(r_sym, deltaQ_sym);
    
    % Numerical values of symbolic variables at this time step
    alp_num = distal.contact.crdCyl.alp;
    phi_num = distal.contact.crdCyl.phi;
    q_num = finger.q; % (4,1)
    
    fkVars_Initial = [alp_num, phi_num, transpose(q_num)];
    
    % Slack Variables
    sp_sym = sym('sp',size(p_sym)); % (3,1), [sp1; sp2; sp3], slack variables in Cartesian position
    sr_sym = sym('sr',size(r_sym)); % (3,1), [sr1; sr2; sr3], slack variables in radial direction
    slack_sym = transpose(cat(1, sp_sym, sr_sym)); % (1,6), slack variables
    slack_num = zeros(size(slack_sym)); % (1,6)
    
    X_key = cat(2, fkVars_sym, slack_sym); % [alp14, phi14, q11, q12, q13, q14, sp1, sp2, sp3, sr1, sr2, sr3], (1, 6+6)
    fkVars_idx = ismember(X_key, fkVars_sym);
    
    %% Iteration of QP
    tic;
    fkVars_num = fkVars_Initial; % Old numerical values of optimization vatiables
    
    % Check initial error
    initial_position = double(subs(p_sym, X_key(fkVars_idx), fkVars_num));
    initial_radial = double(subs(r_sym, X_key(fkVars_idx), fkVars_num));
    pErr = norm(tgt_position(:) - initial_position(:));
    rErr = norm(tgt_normal(:) - initial_radial(:));
    fprintf('Initial error in Cartesian position: %d\n', pErr);
    fprintf('Initial error in radial direction: %d\n', rErr);
            
    for i = 1:nIteration
        % x0 = zeros(size(X_key)); % Initial guess. Not useful for the interior-point-convex algorithm
        x0 = cat(2, fkVars_num, slack_num);
        
        p_num = double(subs(p_sym, fkVars_sym, fkVars_num));
        r_num = double(subs(r_sym, fkVars_sym, fkVars_num));

        Jp_num = double(subs(Jp_sym, fkVars_sym, fkVars_num));
        Jr_num = double(subs(Jr_sym, fkVars_sym, fkVars_num));

        %% Boundary Constraints (boundary of incremental)
        lb_fkVars = [0, -pi, finger.lb] - fkVars_num; % [alp14, phi14, q11, q12, q13, q14]
        ub_fkVars = [1,  pi, finger.ub] - fkVars_num;

        lb_slack = -Inf*ones(size(slack_sym));
        ub_slack =  Inf*ones(size(slack_sym));

        lb = cat(2, lb_fkVars, lb_slack);
        ub = cat(2, ub_fkVars, ub_slack);

        %% Use Linear Constraints [Error: Should be Nonlinear Equality Constraints]
        % Linear Equality Constraints
        Aeq = cat(2, cat(1,Jp_num,Jr_num), -eye(numel(slack_sym))); % (6,12)
        beq = cat(1,tgt_position(:),tgt_normal(:)) - cat(1,p_num(:),r_num(:)); % (6,1)
        
        % Linear Inequality Constraints
        A = [];
        b = [];

        %% Use linear inequality constraints
        %{
        Aeq = [];
        beq = [];
        A = cat(2, cat(1,Jp_num,Jr_num), -eye(numel(slack_sym))); % (6,12)
        b = cat(1,tgt_position(:),tgt_radial(:)) - cat(1,p_num(:),r_num(:)) + 0.01*ones(6,1); % (6,1)
        %}

        %% Objective Function
        R = eye(numel(fkVars_sym)); % damping matrix
        Q = eye(numel(slack_sym)); % for slack variables
        
        %{
        R(4:6,4:6) = 0; % remove radial direction
        Q = zeros(numel(slack_sym)); % disable the penalty for slack variables
        %}
        
        H = blkdiag(2.0*R, 2.0*Q); % Quadratic matrix
        f = zeros(size(X_key)); % Linear component

        options = optimoptions('quadprog','Display','iter');

        [X_sol,fval,exitflag,~,~] = quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options);
        
        if exitflag > 0
            % If solution exists, update numerical values
            X_sol = transpose(X_sol);
            fkVars_num = X_sol(fkVars_idx);
            % slack_num = X_sol(slack_idx);
            
            alp_num = alp_num + fkVars_num(idx_alp);
            phi_num = phi_num + fkVars_num(idx_phi);
            q_num = q_num + fkVars_num(idx_q);
            
            sol_position = double(subs(p_sym, X_key(fkVars_idx), fkVars_num));
            sol_radial = double(subs(r_sym, X_key(fkVars_idx), fkVars_num));

            pErr = norm(tgt_position(:) - sol_position(:));
            rErr = norm(tgt_normal(:) - sol_radial(:));

            fprintf('\nIteration %d\n', i);
            fprintf('Error in Cartesian position: %d\n', pErr);
            fprintf('Error in radial direction: %d\n', rErr);
        else
            disp('No solution found.');
            %{
            pErr = NaN;
            rErr = NaN;
            if nargout > 1
                ikSol = []; % Empty solution
            end
            %}
        end
    end

    t = toc;
    fprintf('Time elapsed: %d\n', t);

    %% Update final solutions
    % fkValues = fkVars_Initial + fkVars_num(:).'; % this is the real value: old_values + solution values (incremental of old values)
    fkValues = fkVars_num; % Using iterative process, no need add initial values
    
    finger.Link{end-1}.contact.crdCyl.alp = fkValues(idx_alp);
    finger.Link{end-1}.contact.crdCyl.phi = fkValues(idx_phi);
    finger.q = fkValues(idx_q);
    
    sol_position = double(subs(p_sym, X_key(fkVars_idx), fkValues));
    sol_radial = double(subs(r_sym, X_key(fkVars_idx), fkValues));

    pErr = norm(tgt_position(:) - sol_position(:));
    rErr = norm(tgt_normal(:) - sol_radial(:));
    fprintf('Error in Cartesian position: %d\n', pErr);
    fprintf('Error in radial direction: %d\n', rErr);

    if nargout > 1
        ikSol.X_key = X_key; % symbolic variables
        ikSol.X_sol = fkValues;
        ikSol.sol_position = sol_position;
        ikSol.sol_radial = sol_radial;
    end
    
    Scores.qp.time(end+1) = t;
    Scores.qp.fval(end+1) = fval;
    Scores.qp.pErr(end+1) = pErr;
    Scores.qp.rErr(end+1) = rErr;
end