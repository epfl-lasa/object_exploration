% This function calculates the weighted IK of a finger model.
% Input:
% - tgt_position: (1,3), desired position of the fingertip
% - tgt_normal: (1,3), desired orientation of the fingertip

function [finger, ikSol] = solveFingerIK(finger, tgt_position, tgt_normal)

    global Scores
    distal = finger.Link{end-1}; % this is the last finger phalanx

    sym_position = distal.contact.symbolic.p; % contact point on fingertip
    sym_radial = distal.contact.symbolic.r; % Surface normal direction (radial direction of finger distal cylinder)

    % Manipulability index: Calc. manipulator Jacobian: J = jacobian(FK, Q)
    EE = finger.Link{end}.symbolic.HT_next(1:3,4);
    J = jacobian(EE, finger.q_sym); % (3, 4)
    squaredOmegas = cumprod(diag(J*transpose(J)));
    sym_metric = squaredOmegas(end); % Last cumulative product value

    % Substitutes constants: L and rho (distance to cylinder surface)
    sym_position = subs(sym_position, {'L', ['rho',num2str(finger.idx),'4']}, [distal.L, distal.contact.crdCyl.rho]);
    sym_radial = subs(sym_radial, {'L', ['rho',num2str(finger.idx),'4']}, [distal.L, distal.contact.crdCyl.rho]);
    sym_metric = subs(sym_metric, {'L', ['rho',num2str(finger.idx),'4']}, [distal.L, distal.contact.crdCyl.rho]);

    X_key = union(union(symvar(sym_position), symvar(sym_radial)), symvar(sym_metric));
    % X_key should be: [alpf4, phif4, qf1, qf2, qf3, qf4] (E.G. FOR FINGER f)
    % disp(X_key);

    idx_alp = ismember(X_key, ['alp',num2str(finger.idx),'4']);
    idx_phi = ismember(X_key, ['phi',num2str(finger.idx),'4']);
    idx_q = ismember(X_key, finger.q_sym);
    
    % Optimization variable: X0 is the initial guess
    X0 = zeros(size(X_key));
    lb = zeros(size(X_key));
    ub = zeros(size(X_key));

    X0(idx_alp) = distal.contact.crdCyl.alp; % alp
    X0(idx_phi) = distal.contact.crdCyl.phi; % phi
    X0(idx_q) = finger.q; % (4,1) initial guess

    lb(idx_alp) = 0;
    % lb(idx_alp) = 1; % force alpha to be 1 (contact at fingertip)
    ub(idx_alp) = 1;
    
    lb(idx_phi) = -pi;
    ub(idx_phi) = pi;
    
    lb(idx_q) = finger.lb;
    ub(idx_q) = finger.ub;

    % Linear constraints
    A = [];
    b = [];
    Aeq = [];
    beq = [];

    nonlcon = [];

    initGuess = X0;

    % Weighting coefficients for components of objective function: (1) position, (2) contact normal, (3) metric, (4) damping
    Weights = normalize([1,1,0,1],'norm');

    symIKObjectiveFunction(X_key, initGuess, sym_position, sym_radial, sym_metric, tgt_position, tgt_normal, Weights);

    objfun = @(X)ikObjfun(X);

    options = optimoptions('fmincon',...
        'Algorithm','sqp',...% 'active-set', 'interior-point', 'sqp', 'trust-region-reflective', or 'sqp-legacy'
        'SpecifyObjectiveGradient',false,... % No gradient for objective function available
        'SpecifyConstraintGradient',false,...
        'Display','final');

    tic;
    [X_sol, fval, exitflag, ~] = fmincon(objfun, X0, A, b, Aeq, beq, lb, ub, nonlcon, options);
    t = toc;
    fprintf('Time elapsed: %d\n', t);
    
    if exitflag > 0
        sol_position = double(subs(sym_position, X_key(:), X_sol(:)));
        sol_normal = double(subs(sym_radial, X_key(:), X_sol(:)));
        sol_metric = double(subs(sym_metric, X_key(:), X_sol(:)));
        
        pErr = norm(tgt_position(:) - sol_position(:));
        rErr = norm(tgt_normal(:) - sol_normal(:));
        fprintf('Error in Cartesian position: %d\n', pErr);
        fprintf('Error in radial direction: %d\n', rErr);

        % Update finger parameters
        distal.contact.crdCyl.alp = X_sol(idx_alp); % alp
        distal.contact.crdCyl.phi = X_sol(idx_phi); % phi
        finger.q = X_sol(idx_q); % (4,1) initial guess
        
        finger.Link{end-1} = distal;
        
        if nargout > 1
            ikSol.X_key = X_key; % symbolic variables
            ikSol.X_sol = X_sol;    
            ikSol.sol_position = sol_position;
            ikSol.sol_normal = sol_normal;
            ikSol.sol_metric = sol_metric;
        end
    else
        pErr = NaN;
        rErr = NaN;
        disp('No solution found.');
        if nargout > 1
            ikSol = []; % Empty solution
        end
    end

    Scores.fmincon.time(end+1) = t;
    Scores.fmincon.fval(end+1) = fval;
    Scores.fmincon.pErr(end+1) = pErr;
    Scores.fmincon.rErr(end+1) = rErr;
end


function symIKObjectiveFunction(X_key, initGuess, sym_position, sym_normal, sym_metric, tgt_position, tgt_normal, Weights)
    % This function calculates the symbolic expression of the objective
    % function and saves it locally as a MATLAB function file.
    % initX: initial value of X, taken from the current hand configuration
    
    delta_position = sym_position(:) - tgt_position(:);
    delta_normal = sym_normal(:) - tgt_normal(:);
    
    % damping term
    deltaX = X_key(:) - initGuess(:); % (6,1)
    Lambda = diag(ones(size(X_key))); % (6,6) weighting matrix for each variable
    
    obj = dot(Weights, ...
        [transpose(delta_position)*delta_position,... % position
        transpose(delta_normal)*delta_normal,... % contact normal (radial direction)
        norm(1./sym_metric, 2),... % to maximize manipulability
        transpose(deltaX)*Lambda*deltaX]); % damping
    
    matlabFunction(obj,'File','ikObjFun','Vars',X_key,'Optimize',false);
    
    % obj_grad = transpose(jacobian(obj,X_key));
    % matlabFunction(obj_grad,'File','ikObjFunGrad','Vars',X_key,'Optimize',false);
end


function [f, gradf] = ikObjfun(X)
    % This is the interface of objective function
    
    % if sum(isnan(X),'all') % No need for sqp
    %     disp('X contains NaN.');
    %     disp(X);
    % end
    Xcell = num2cell(X);
    
    f = ikObjFun(Xcell{:}); % Value
    % whos('f');
    
    if nargout > 1
        gradf = ikObjFunGrad(Xcell{:}); % Gradient
        % whos('gradf');
    end
end