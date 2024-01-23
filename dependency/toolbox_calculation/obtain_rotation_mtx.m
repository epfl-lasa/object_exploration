%%% Axis-angle representation (u,theta) to rotation matrix R
% u is the rotation axis, normal vector
% theta in radius

function [R,dRdx,dRdy,dRdz,dRdt] = obtain_rotation_mtx(u, theta)
    if norm(u)~=1
        u = u/norm(u);
    end
    R = [cos(theta)+u(1)^2*(1-cos(theta)), u(1)*u(2)*(1-cos(theta))-u(3)*sin(theta), u(1)*u(3)*(1-cos(theta))+u(2)*sin(theta);...
        u(2)*u(1)*(1-cos(theta))+u(3)*sin(theta), cos(theta)+u(2)^2*(1-cos(theta)), u(2)*u(3)*(1-cos(theta))-u(1)*sin(theta);...
        u(3)*u(1)*(1-cos(theta))-u(2)*sin(theta), u(3)*u(2)*(1-cos(theta))+u(1)*sin(theta), cos(theta)+u(3)^2*(1-cos(theta))];

    x = u(1);
    y = u(2);
    z = u(3);
    if nargin>1 % return derivative
        dRdx = [ -2*x*(cos(theta) - 1), -y*(cos(theta) - 1), -z*(cos(theta) - 1);...
            -y*(cos(theta) - 1),                   0,         -sin(theta);...
            -z*(cos(theta) - 1),          sin(theta),                   0];
        dRdy = [                   0,   -x*(cos(theta) - 1),          sin(theta);...
            -x*(cos(theta) - 1), -2*y*(cos(theta) - 1), -z*(cos(theta) - 1);...
            -sin(theta),   -z*(cos(theta) - 1),                   0];
        dRdz = [                   0,         -sin(theta),   -x*(cos(theta) - 1);...
            sin(theta),                   0,   -y*(cos(theta) - 1);...
            -x*(cos(theta) - 1), -y*(cos(theta) - 1), -2*z*(cos(theta) - 1)];
        dRdt = [   sin(theta)*x^2 - sin(theta), x*y*sin(theta) - z*cos(theta), y*cos(theta) + x*z*sin(theta);...
            z*cos(theta) + x*y*sin(theta),   sin(theta)*y^2 - sin(theta), y*z*sin(theta) - x*cos(theta);...
            x*z*sin(theta) - y*cos(theta), x*cos(theta) + y*z*sin(theta),   sin(theta)*z^2 - sin(theta)];
    end

    
end