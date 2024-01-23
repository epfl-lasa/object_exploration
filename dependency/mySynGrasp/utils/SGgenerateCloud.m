% This function generates a cloud of N points along a sphere with center
% given by object's matrix H, with gauge G. Then creates an array of Homogeneous
% transformation matrices for hand pose
% P is a 4*4*N matrix Sigma-World R.F.
%  This file is part of SynGrasp (Synergy Grasping Toolbox).
%
%  Copyright (c) 2013, M. Malvezzi, G. Gioioso, G. Salvietti, D. Prattichizzo,
%  All rights reserved.
% 
%  Redistribution and use with or without
%  modification, are permitted provided that the following conditions are met:
%      * Redistributions of source code must retain the above copyright
%        notice, this list of conditions and the following disclaimer.
%      * Redistributions in binary form must reproduce the above copyright
%        notice, this list of conditions and the following disclaimer in the
%        documentation and/or other materials provided with the distribution.
%      * Neither the name of the <organization> nor the
%        names of its contributors may be used to endorse or promote products
%        derived from this software without specific prior written permission.
% 
%  THIS SOFTWARE IS PROVIDED BY M. Malvezzi, G. Gioioso, G. Salvietti, D.
%  Prattichizzo, ``AS IS'' AND ANY
%  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
%  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
%  DISCLAIMED. IN NO EVENT SHALL M. Malvezzi, G. Gioioso, G. Salvietti, D.
%  Prattichizzo BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
%  EXEMPLARY, OR CONSEQUENTIAL DAMAGES
%  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
%  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
%  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
%  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
%  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


 function P = SGgenerateCloud(struct,G,N)

 switch struct.type
     case 'sph'
         G = struct.radius + G;
     case 'cyl'
         dist = norm(struct.p(:,1,1) - struct.center);
         G = dist + G;
     case 'cube'
         G = norm(0.5*(struct.dim)) + G;
     otherwise
         error('bad input argument')
 end
         
% These are realizations of Random Variables ~U[0,2*pi]
x_angle = 2*pi.*rand(1,N); % parameter for SGrotx 
y_angle = 2*pi.*rand(1,N); % parameter for SGroty
z_angle = 2*pi.*rand(1,N); % parameter for SGrotz

% Rotations:
Rx = zeros(3,3,N);
Ry = Rx;
Rz = Rx;
H_tmp = zeros(4,4,N);
P = H_tmp;
for i = 1:N
    H_tmp(:,4,i) = [struct.center;1];
    Rx(:,:,i) = SGrotx(x_angle(i));
    Ry(:,:,i) = SGroty(y_angle(i));
    Rz(:,:,i) = SGrotz(z_angle(i));
    H_tmp(1:3,1:3,i) = Rx(:,:,i)*Ry(:,:,i)*Rz(:,:,i);
    H_tmp(:,:,i) = H_tmp(:,:,i)*SGtransl([0,0,G]);
    %H_tmp(:,:,i) = H_tmp(:,:,i)*SGtransl([0,0,0]);
    P(:,:,i) = H_tmp(:,:,i);
end