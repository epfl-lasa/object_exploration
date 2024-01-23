function evaluateOptimizationResults(X_sol, param)
% This function calculates the value of constraints to see which
% constraints are satisfied and which are not, given the solution of
% optimization problem, X_sol
Xcell = num2cell(X_sol);

%% Nonlinear Inequality Constraints
fprintf('\n* Inequality Constraint: ');
c = nonl_c(Xcell{:});
c = c(:).';
c_idx = param.c_idx;
c_name = param.c_name;

nc = numel(c_name); % number of constraint types

if ~isequal(length(c),sum(c_idx))
    warning('Incorrect length: c');
    disp(c_name);
else
    L = 0; % length counter
    for i = 1:nc
        disp(c_name{i});
        c_temp = c(L+1: L+c_idx(i));
        if ~all(c_temp<=0)
            disp(c_temp);
        end
        L = L + c_idx(i);
    end
end

%% Nonlinear Equality Constraints
fprintf('\n* Equality Constraint: ');
ceq = nonl_ceq(Xcell{:});
ceq = ceq(:).';
ceq_idx = param.ceq_idx;
ceq_name = param.ceq_name;

nceq = numel(ceq_name); % number of constraint types

if ~isequal(length(ceq),sum(ceq_idx))
    warning('Incorrect length: ceq');
    disp(ceq_name);
else
    L = 0; % length counter
    for i = 1:nceq
        disp(ceq_name{i});
        ceq_temp = ceq(L+1: L+ceq_idx(i));
        if any(ceq_temp)
            disp(ceq_temp);
        end
        L = L + ceq_idx(i);
    end
end
end