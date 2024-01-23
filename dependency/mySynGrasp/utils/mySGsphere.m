%    Arguments:
%    H = homogeneous transformation
%    rad = radius
%    res = number of points on the external surface

function struct = mySGsphere(Htr,rad,clr,res)
if nargin < 4
    res = 20;
end
if nargin < 3
    % clr = 'none'; % default color of sphere
    clr = [127/255 1 212/255]; % aquamarine
end
if nargin < 2
    rad = 25;
end
struct.type = 'sph';
struct.tag = [struct.type,'_r_',num2str(rad)]; % specifications to identify the object

struct.res = res;
struct.clr = clr;
struct.Htr = Htr;
struct.center = Htr(1:3,4);
struct.radius = rad;
[X,Y,Z] = sphere(res); % creates a unit sphere mesh

struct.p = zeros(3,size(X,2),size(X,1));
for k = 1:size(X,1)
    for j = 1:size(X,2)
        p = [rad*[X(k,j);Y(k,j);Z(k,j)];1]; % transform to homogeneous coordinates
        v = Htr*p;
        struct.p(:,j,k) = v(1:3);
    end
end

for i = 1:size(X,1)
    for j = 1:size(X,2)
        struct.X(i,j) = struct.p(1,j,i);
        struct.Y(i,j) = struct.p(2,j,i);
        struct.Z(i,j) = struct.p(3,j,i);
    end
end

%% Create symbolic expression of optimization variables that belong to object
syms x y z t1 t2 t3 t4
struct.symbolic.cntr = [x, y, z]; % For sphere object, using center only is enough
struct.symbolic.quat = [t1 t2 t3 t4]; % not used. just follow the convention of defining object
end