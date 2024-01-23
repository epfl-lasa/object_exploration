% Calculate the area of triangle given vertices (X,Y,Z)
function S = areaTriangle(A,B,C)
    % https://www.quora.com/How-can-I-find-the-area-of-a-triangle-in-3D-coordinate-geometry#:~:text=X1%2C0).-,Here%20X1%20is%20length%20of%20AB.,use%20two%20dimensional%20coordinate%20system.

    if nargin < 1
        A = [sym('ax'),sym('ay'),sym('az')];
        B = [sym('bx'),sym('by'),sym('bz')];
        C = [sym('cx'),sym('cy'),sym('cz')];
        
        X = sum((A-B).^2);
        Y = sum((B-C).^2);
        Z = sum((C-A).^2);
        S = sqrt(4*X*Y - (Z-X-Y)^2)/4;
    else
        A = A(:).';
        B = B(:).';
        C = C(:).';
        
        if isequal(A,B) || isequal(B,C) || isequal(C,A) % degenerated triangle
            S = 0;
        else
            X = sum((A-B).^2);
            Y = sum((B-C).^2);
            Z = sum((C-A).^2);
            S = sqrt(4*X*Y - (Z-X-Y)^2)/4;
        end
    end
end