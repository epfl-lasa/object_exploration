function plotHomoTrans(H, clr, ls)
% Plot the standard reference frame transformed by the homogeneous transformation matrix 'H' in color 'clr'
% Input: ls: line style
if nargin < 3 || isempty(ls)
    ls = '-';
end
if nargin < 2 || isempty(clr)
    clr = ['r','g','b'];
end

if length(clr)==1
    clr = repmat(clr,1,3);
end
if length(ls)==1
    ls = repmat(ls,1,3);
end

X = H*[1;0;0;1]; % Transformed X, using the homo.coordinate of X
Y = H*[0;1;0;1];
Z = H*[0;0;1;1];
O = H(1:3,4); % Origin

% [REMARK] The following way of plotting is wrong. The reason is, X is the vector in the current CF (absolute value), instead of direction vectors.
%{
quiver3(O(1),O(2),O(3),X(1),X(2),X(3),[clr(1),ls(1)]);
quiver3(O(1),O(2),O(3),Y(1),Y(2),Y(3),[clr(2),ls(2)]);
quiver3(O(1),O(2),O(3),Z(1),Z(2),Z(3),[clr(3),ls(3)]);
%}

% The following is the correct code:
quiver3(O(1),O(2),O(3),X(1)-O(1),X(2)-O(2),X(3)-O(3),[clr(1),ls(1)]);
quiver3(O(1),O(2),O(3),Y(1)-O(1),Y(2)-O(2),Y(3)-O(3),[clr(2),ls(2)]);
quiver3(O(1),O(2),O(3),Z(1)-O(1),Z(2)-O(2),Z(3)-O(3),[clr(3),ls(3)]);

%%% Implementation using plot3 function
%{
Vx = [O,X(1:3)]'; % (3,2) -> (2,3)
Vy = [O,Y(1:3)]';
Vz = [O,Z(1:3)]';
plot3(Vx(:,1),Vx(:,2),Vx(:,3),[clr(1),ls(1)],...
Vy(:,1),Vy(:,2),Vy(:,3),[clr(2),ls(2)],...
Vz(:,1),Vz(:,2),Vz(:,3),[clr(3),ls(3)]);
%}

hold on;

end