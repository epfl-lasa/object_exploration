% This is the objective function for the grasp optimization in joint space
function [f, gradf] = optGraspJS_objfun(X)

    if sum(isnan(X),'all')
        disp('X contains NaN.');
    end
    Xcell = num2cell(X);
    
    f = objfun(Xcell{:}); % Value
    
    if nargout > 1
        gradf = objfun_grad(Xcell{:}); % Gradient
    end
end