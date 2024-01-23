% Convert homogeneous transformation, T, to pose X.
% T: homogeneous transformation
% X: pose, (6,1), first 3: position, last 3: exponential map
% 
% Used function: RotationMatrix, AlbeCrive (2020). Rotation Matrix 3D (https://www.mathworks.com/matlabcentral/fileexchange/46419-rotation-matrix-3d), MATLAB Central File Exchange. Retrieved June 18, 2020.

function X = tform2pose(T)

    eps = 1e-10;

    X = zeros(6,1);
    pos = T(1:3,4);
    rot = T(1:3,1:3);

    exp_map = RotationMatrix(rot, 'rotationMatrix').GetExponentialMap();
    assert(max(abs(exp_map - [pi;0;0])) < eps); % Check exponential map converted from diagonal mtx
    assert(max(abs(exp_map - [0;pi;0])) < eps);
    assert(max(abs(exp_map - [0;0;pi])) < eps);

    X(1:3) = pos;
    X(4:6) = exp_map;

end