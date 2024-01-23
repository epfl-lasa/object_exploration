function S = calcPolygonArea(X, p)
    % p is an arbitrary point inside the polygon (or outside), as a reference
    % X (N,3) contains all vertices points

    if nargin<2
        p = [sym('px'),sym('py'),sym('pz')];
    end
    if isa(p,'sym')
        X = sym(X);
    end
    
    N = size(X,1); % number of points
    if N == 2
        A = X(1,:);
        B = X(2,:);
        C = p;
        S = calcTriangleArea2D(A,B,C);
    elseif N > 2
        S = 0;
        C = p;
        for i = 1:N
            A = X(i,:);
            if i < N
                B = X(i+1,:);
            else
                B = X(1,:);
            end
            S = S + areaTriangle(A,B,C);
        end
    else
        disp(N);
        error('Wrong dimension of input data dimension.');
    end
end