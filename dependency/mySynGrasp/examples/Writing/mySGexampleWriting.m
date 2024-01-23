%  SGexampleWriting - SynGrasp demo concerning the anthropomorphic hand
%  writing
%
%       Usage: SGexampleWriting
%
%       References: 
%       1. D. Prattichizzo, M. Malvezzi, Evaluating human hand  
%       manipulability performance during handwriting tasks, presented
%       at the workshop "Hand Synergies: how to tame the complexity of
%       grasping", ICRA 2013
%       2. D. Prattichizzo, L. Meli, M. Malvezzi. Digital Handwriting with 
%       a Finger or a Stylus: a Biomechanical Comparison. IEEE Transactions 
%       on Haptics, 8(4):356-370, 2015.
% 
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

close all;
clear all;
clc;
    
hand = mySGparadigmatic;

[qm, S] = mySGsantelloSynergies;

hand = mySGdefineSynergies(hand,S,1:15);

%%%% choose the writing configuration
hand = mySGhumanWritingConf(hand);

%%%% contact points

% thumb and index fingertip
hand = mySGaddFtipContact(hand,1,1:2);
% DP joint of the middle finger
hand = mySGaddContact(hand,1,3,3,1); % mySGaddContact(hand,type,cwhere,link,alpha)

% type = a flag indicating the contact type with the following semantics:
%   1: hard finger (2D or 3D);
%   2: soft finger (2D or 3D);
%   3: complete constraint (2D or 3D);
% cwhere = the finger on which the contact point lies
% link = the link where the contact point lies
% alpha = an argument that parameterize the position of the contact point on the link (0 = link base; 1 = link end)

% define the pencil
[hand,object] = mySGmakeObject(hand);

object.center(1) = object.center(1)+12;
object.center(3) = object.center(3)-30;
object.center(2) = object.center(2)+10;

%%%%% plot the hand, contact points and contact normals
figure(2)
view([-144 10]);

hold on
% plot the contact points and contact normals
plot3(object.center(1),object.center(2),object.center(3),'rd','LineWidth',3,'MarkerSize',8)
hold on
grid on
for i = 1:size(hand.cp,2)
    % assign the contact point to the respective finger
    plot3(hand.cp(1,i),hand.cp(2,i),hand.cp(3,i),'m*','Linewidth',2,'MarkerSize',8)
    quiver3(hand.cp(1,i),hand.cp(2,i),hand.cp(3,i),object.normals(1,i),object.normals(2,i),object.normals(3,i),10,'LineWidth',2)
end
axis('equal')
out = mySGdefinePencil(object);
mySGplotHand(hand)

%% [K.Yao] Test: Validation of force closure property
%{
P = hand.cp(1:3,:); % (3,N)
V = object.normals; % (3,N)
mu = 0.3;
k = 10;
ctr = object.center;
flag = forceClosureCheck(P, V, mu, k, ctr);
fprintf('Force closure grasp: %d\n', flag);
%}

% re - define hand Jacobian and grasp matrix in the new configuration
hand.Jtilde = mySGjacobianMatrix(hand);
H = mySGselectionMatrix(object);
object.H = H;
hand.H = H;
hand.J = H*hand.Jtilde;
object.Gtilde = mySGgraspMatrix(object);
object.G = object.Gtilde*transpose(hand.H);

[nl,nq]= size(hand.J);
[nd]= size(object.G,1);

% choose the synergy indexes
syn_index = 1:6;

% choose the corresponding columns
S_rid = S(:,syn_index);
hand.S = S_rid;
nz = size(hand.S,2);


% define the stiffness matrices
Ks = eye(nl);
Kq = eye(nq);
Kz = eye(nz);

object = mySGcontactStiffness(object,Ks);
hand = mySGjointStiffness(hand,Kq);
hand = mySGsynergyStiffness(hand,Kz);

%%%%% constant synergy matrix
Ksz = zeros(nz,nz);
Kjq = zeros(nq,nq);
Kju = zeros(nq,nd);

% evaluate the homogeneous quasi static solution
Gamma = mySGquasistaticHsolution(hand,object);

% evaluate the kinematic manipulability ellipsoid
kinmanipulability = mySGkinManipulability(Gamma);

% translate the kinematic ellipsoid on the pen tip
kinellips = kinmanipulability.kinellips;
[r,c] = size(kinellips.u1);
for i = 1:r
    for j = 1:c
        u1t(i,j) = kinellips.u1(i,j)+object.center(1);
        u2t(i,j) = kinellips.u2(i,j)+object.center(2);
        u3t(i,j) = kinellips.u3(i,j)+object.center(3);
    end
end

% draw the kinematic manipulability ellipsoid in the workspace
figure(2)
hold on
axis([-60 30 40 120 -100 50])
mesh(u1t,u2t,u3t)