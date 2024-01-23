% Calculate the volume of tetrahedron given vertices A, B, C, D
function v = volumeTetrahedron(A,B,C,D)
% Input: A, B, C, D, are four indices. Each one is of size (3,1)
% https://en.wikipedia.org/wiki/Tetrahedron

    v = abs(dot(B-A, cross(C-A, D-A)))/6;
%     Notice that abs(X) can be written as sqrt(power(X,2)) to calculate
%     derivative

end