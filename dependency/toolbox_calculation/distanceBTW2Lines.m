function d = distanceBTW2Lines(x1,x2,y1,y2,N)
% This function calculates distance between two lines X (x1->x2) and Y (y1->y2), sepcified by
% pair of 3D points (x1, x2) and (y1 ,y2), respectively. x1,x2 are
% end-points of line segment X; y1,y2 are end-points of line segment Y.
% All data points are column vectors in shape (3,1)
% Output:
% - d: the minimum distance
% - s: the vector between the proximate points
% Reference: https://homepage.univie.ac.at/Franz.Vesely/notes/hard_sticks/hst/hst.html

if nargin < 5
    N = 5;
end

if isequal(x1,x2) && ~isequal(y1,y2) % link X is virtual
    d = norm(cross(x1-y1,x1-y2))/norm(y2-y1);
elseif ~isequal(x1,x2) && isequal(y1,y2) % link Y is virtual
    d = norm(cross(y1-x1,y1-x2))/norm(x2-x1);
elseif isequal(x1,x2) && isequal(y1,y2) % both links are virtual
    d = norm(x1-y1);
else
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
    
    %%% Sampling based calculation of distance between 3d line segments
    if isa(x1,'sym')
        d = sym([]);
    elseif isa(x1,'double')
        d = [];
    end
    
    for i = 0:(N-1) % 5 sampling data points
        y0 = y1 + (y2-y1)*i/N;
        d(end+1) = norm(cross(y0-x1,y0-x2))/norm(x2-x1);
    end
end

end