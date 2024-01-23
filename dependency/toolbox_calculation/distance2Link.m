function [d_min, ratio] = distance2Link(pnt, link, N)
% Calculate the minimum distance from a given point, 'pnt' (3,1), to a
% finger link using interpolation.
% Input:
%   - pnt: spatial point, (3,1)
%   - link: target link. see 'makeLink'
%   - N: number of interpolation points.
% Output:
%   - d_min: minimum distance from the point to the link
%   - ratio: ratio of the min_distance point on the link, to the end of the
%   link. e.g. if the 1/3 part of the link is closest to the pnt, then ratio = 1/3 

if nargin < 3
    N = 100;
end

p0 = link.HT_this(1:3,4);
p1 = link.HT_next(1:3,4);

t = repmat(linspace(0,1,N), 3,1); % expand to three dimensions of the point
p_list = (1-t).*p0 + t.*p1; % (3,N)

d = pdist2(p_list', pnt','euclidean');
[d_min, idx_min] = min(d);

ratio = idx_min/N;