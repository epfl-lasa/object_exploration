% Convert pose X, to homogeneous transformation, T.
% X: pose, (6,1), first 3: position, last 3: exponential map
% T: homogeneous transformation
% 
% Used function: RotationMatrix, AlbeCrive (2020). Rotation Matrix 3D (https://www.mathworks.com/matlabcentral/fileexchange/46419-rotation-matrix-3d), MATLAB Central File Exchange. Retrieved June 18, 2020.

function T = pose2tform(X)

    T = eye(4);
    pos = X(1:3);

    expmap = X(4:6);
    rot = RotationMatrix(expmap, 'exponentialMap').GetRotationMatrix();

    T(1:3,4) = pos;
    T(1:3,1:3) = rot;
    
end