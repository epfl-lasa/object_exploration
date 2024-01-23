%    Htr = Homogeneous transformation
%    h = height of the cylinder
%    rad = radius
%    res = number of points around circumference

function struct = mySGcylinder(Htr,height,rad,clr,res)

if nargin < 5
    res = 50;
end
if nargin < 4
    clr = [127/255 1 212/255]; % aquamarine
end
if nargin < 3
    rad = 10;
end
if nargin < 2
    height = 50;
end
if nargin < 1
    Htr = eye(4);
end

[X,Y,Z] = cylinder(rad,res);
Z = height*Z; %height accordingly to requested

struct.p = zeros(3,size(X,2),size(X,1));
for k = 1:size(X,1)
    for j = 1:size(X,2)
        p = [X(k,j);Y(k,j);Z(k,j);1];
        v = Htr*SGtransl([0,0,-height/2])*p;
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

struct.Htr = Htr;
struct.radius = rad;
struct.height = height;
struct.res = res;

struct.clr = clr;
struct.type = 'cyl';
struct.tag = [struct.type,'_r_',num2str(rad),'_h_',num2str(height)];

HtrC1 = Htr*SGtransl([0,0,-height/2]);
HtrC2 = Htr*SGtransl([0,0, height/2]);

c1 = HtrC1(1:3,4);
c2 = HtrC2(1:3,4);

struct.c1 = c1;
struct.c2 = c2;

struct.HtrC1 = HtrC1(1:3,4);
struct.HtrC2 = HtrC2(1:3,4);

struct.axis = (c1-c2)/norm(c1-c2); % central axis direction
struct.center = Htr(1:3,4);

%% Create symbolic expression of optimization variables
syms x y z
symCntr = sym([x, y, z]);

%%% Using Quaternions
syms t1 t2 t3 t4
symQuat = sym([t1, t2, t3, t4]);
symRotm = symQuat2Rotm(symQuat); % rotation matrix
symHtr = [symRotm, symCntr(:);...
    0,0,0,1];

%%% Calculate two ends
symC1 = symHtr*SGtransl([0,0,-height/2]); % (4,4)
symC2 = symHtr*SGtransl([0,0, height/2]); % (4,4)
symAxis = (symC1(1:3,4)-symC2(1:3,4))/norm(symC1(1:3,4)-symC2(1:3,4));

struct.symbolic.cntr = symCntr;
struct.symbolic.quat = symQuat;
struct.symbolic.Htr = symHtr;

struct.symbolic.symC1 = symC1;
struct.symbolic.symC2 = symC2;

struct.symbolic.c1 = symC1(1:3,4);
struct.symbolic.c2 = symC2(1:3,4);
struct.symbolic.axis = symAxis;
end