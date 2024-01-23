% Determine if two arrays are equal considering numerical error
function [flag, err] = isNumericallyEqual(X,Y,tol)
    if nargin < 3
        tol = 1e-6; % tolerance of numerical round-off error
    end
    one = ones(size(Y));
    flag = (X <= Y+tol*one) & (X >= Y-tol*one);
    if nargout > 1
    	err = sum(X-Y,'all');
    end
end