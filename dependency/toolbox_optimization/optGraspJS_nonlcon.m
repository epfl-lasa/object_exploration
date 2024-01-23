% The nonlinear constraint function of the grasp planning in joint space
function [c, ceq, dc, dceq] = optGraspJS_nonlcon(X)

    if sum(isnan(X),'all')
        warning('optGraspJS_nonlcon: X contains NaN.');
    end
    Xcell = num2cell(X);
    
    c = nonl_c(Xcell{:}); % Inequality constraint
    ceq = nonl_ceq(Xcell{:}); % Equality constraint

    if nargout > 2
        dc = nonl_c_grad(Xcell{:}); % Gradient
        dceq = nonl_ceq_grad(Xcell{:});
    end
end