function S = calcTriangleArea2D(A,B,C)
    % A, B, C are three vertices of the triangle
    % S is the area
    % Reference: https://www.mathopenref.com/coordtrianglearea.html
    if nargin < 1 % symbolic form
        A = [sym('ax'),sym('ay')];
        B = [sym('bx'),sym('by')];
        C = [sym('cx'),sym('cy')];
        S = norm(A(1)*(B(2)-C(2))+...
                B(1)*(C(2)-A(2))+...
                C(1)*(A(2)-B(2)))/2;
    else
        A = A(:);
        B = B(:);
        C = C(:);
        if isequal(A,B) || isequal(B,C) || isequal(C,A) % degenerated triangle
            S = 0;
        else
            S = norm(A(1)*(B(2)-C(2))+...
                B(1)*(C(2)-A(2))+...
                C(1)*(A(2)-B(2)))/2;
        end
    end
end