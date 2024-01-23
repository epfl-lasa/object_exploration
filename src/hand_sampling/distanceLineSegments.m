function d = distanceLineSegments(x1,x2,y1,y2,Nx,Ny,pair_type,isSquared)
% This function calculates distance between two line segments X (x1->x2) and Y (y1->y2), specified by
% pair of 3D points (x1, x2) and (y1 ,y2), respectively. x1,x2 are
% end-points of line segment X; y1,y2 are end-points of line segment Y.
% All data points are column vectors in shape (3,1)
% Output:
% - d: the minimum distance
% - s: the vector between the proximate points
% Reference: https://homepage.univie.ac.at/Franz.Vesely/notes/hard_sticks/hst/hst.html

if nargin < 8
    isSquared = true;
end

if nargin < 7
    pair_type = 'paired';
    % 'paired': distance between paired points from two line segments. E.g.,
    % La = [La1,La2,La3,La4,La5], and Lb = [Lb1,Lb2,Lb3,Lb4,Lb5], this
    % returns pairwise distance: d(La1,Lb1), d(La2,Lb2),..., d(LaN,LbN).
    % This results in N constraints in total. However, in practice, it
    % usually also requires to compute La and Lb_inverted =
    % [Lb5,Lb4,Lb3,Lb2,Lb1]. So, 2*N in total (need to call this function
    % twice.
    
    % pair_type = 'full';
    % 'full': complete distance matrix between any two points from La and
    % Lb, respectively. This calculates d(La1,Lb1), d(La1,Lb2), ...,
    % d(Lai,Lbj), i,j = 1,...,N.
    % This results in N^2 constraints in total.
end

if nargin < 5
    Nx = 5; % number of samples to approximate x1-x2
    Ny = 5; % number of samples to approximate y1-y2
end

x1 = x1(:);
x2 = x2(:);
y1 = y1(:);
y2 = y2(:);

% if N == 1
%     x0 = (x1+x2)/2;
%     y0 = (y1+y2)/2;
%     if isSquared
%         d = (x0-y0).'*(x0-y0);
%     else
%         d = norm(x0-y0);
%     end
% end

if isequal(x1,x2) && isequal(y1,y2) % both links are virtual, degenerate to distance between two 3d points
    if isSquared
        d = (x1-y1).'*(x1-y1);
    else
        d = norm(x1-y1);
    end
    
elseif isequal(x1,x2) && ~isequal(y1,y2) % link X is virtual
    % This is just an approximation. Not accurate.
    % d = norm(cross(x1-y1,x1-y2))/norm(y2-y1);
    
    if isa(y1,'sym') % for symbolic expressions
        d = sym(zeros(1,Ny));
    elseif isa(y1,'double') % for numerical expressions
        d = zeros(1,Ny);
    end
    
    for i = 0:(Ny-1) % Ny sampling data points in total
        % reference: https://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
        assert(Ny>1);
        y0 = y1 + (y2-y1)*i/(Ny-1);
        
        if isSquared
            d(i+1) = (x1-y0).'*(x1-y0); % d is the minimal distance between point y0 (3,1) and line segment [x1,x2]
        else
            d(i+1) = norm(x1-y0);
        end
    end
            
elseif ~isequal(x1,x2) && isequal(y1,y2) % link Y is virtual
    % d = norm(cross(y1-x1,y1-x2))/norm(x2-x1);
    
    if isa(x1,'sym') % for symbolic expressions
        d = sym(zeros(1,Nx));
    elseif isa(x1,'double') % for numerical expressions
        d = zeros(1,Nx);
    end
    
    assert(Nx>1);
    for i = 0:(Nx-1) % Nx sampling data points in total
        % reference: https://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
        
        x0 = x1 + (x2-x1)*i/(Nx-1);
        
        if isSquared
            d(i+1) = (x0-y1).'*(x0-y1); % d is the minimal distance between point y0 (3,1) and line segment [x1,x2]
        else
            d(i+1) = norm(x0-y1);
        end
    end
    
else
    %%% Sampling based calculation of distance between 3d line segments
    switch pair_type
        case 'paired'
            if Nx ~= Ny
                N = ceil(mean([Nx,Ny]));
                % warning('Nx %d and Ny %d are not equal, force N to be %d.\n', Nx, Ny, N);
            else
                N = Nx;
            end

            if isa(x1,'sym') % for symbolic expressions
                d = sym(zeros(1,N));
            elseif isa(x1,'double') % for numerical expressions
                d = zeros(1,N);
            end

            assert(N>1);
            for i = 0:(N-1) % N sampling data points in total
                % reference: https://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
                y0 = y1 + (y2-y1)*i/(N-1);
                % d is the minimal distance between point y0 (3,1) and line segment [x1,x2]
                assert(~isequal(x1,x2));
                if isSquared
                    vec_n = symCross(y0-x1,y0-x2); % (3,1), numerator
                    vec_d = x2-x1; % (3,1), denominator
                    d(i+1) = (vec_n.'*vec_n)/(vec_d.'*vec_d);
                else
                    d(i+1) = norm(symCross(y0-x1,y0-x2))/norm(x2-x1);
                end
            end
            
        case 'full'
            if isa(x1,'sym') % for symbolic expressions
                d = sym(zeros(Nx,Ny));
            elseif isa(x1,'double') % for numerical expressions
                d = zeros(Nx,Ny);
            end
            assert(Nx>1);
            assert(Ny>1);
            for i = 0:(Nx-1)
                for j = 0:(Ny-1)
                    x0 = x1 + (x2-x1)*i/(Nx-1);
                    y0 = y1 + (y2-y1)*j/(Ny-1);
                    
                    assert(~isequal(x0,y0));
                    
                    if isSquared
                        d(i+1,j+1) = (x0-y0).'*(x0-y0);
                    else
                        d(i+1,j+1) = norm(x0-y0);
                    end
                end
            end
            
        otherwise
            error('NotImplementedError.');
    end
end

if isnan(d)
    warning('Distance contains NaN.');
end

end





% distance between two lines
%{
r1 = x1; e1 = x2-x1; e1 = e1./norm(e1); % Line X is specified by: v = r1 + k1*e1;
r2 = y1; e2 = y2-y1; e2 = e2./norm(e2); % Line Y is specified by: w = r2 + k2*e2;
r12 = r2 - r1;
den = 1 - (dot(e1,e2)).^2; % denominator
k1 =  (dot(r12,e1)-dot(r12,e2)*dot(e1,e2))/den;
k2 = -(dot(r12,e2)-dot(r12,e1)*dot(e1,e2))/den;
% vector between two proximate points 
s = r12 + k2*e2 - k1*e1;
d = norm(s);
%}