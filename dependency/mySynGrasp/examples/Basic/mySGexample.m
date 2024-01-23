%    SGexample - SynGrasp demo concerning the anthropomorphic hand
%
%    Usage: SGexample
%
%    The demo includes
%    underactuated hand model, contact point definition, stiffness and 
%    synergy matrices definition, grasp analysis
% 
%  Copyright (c) 2013, M. Malvezzi, G. Gioioso, G. Salvietti, D.
%     Prattichizzo,
%  All rights reserved.
% 
%  Redistribution and use with or without
%  modification, are permitted provided that the following conditions are met:
%      * Redistributions of source code must retain he above copyright
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

close all
clear all
clc

hand = mySGparadigmatic;

[qm, S] = mySGsantelloSynergies;
 
hand = mySGdefineSynergies(hand,S(:,1:4),qm); 
 
figure(1)
mySGplotHand(hand);
hand = mySGmoveHand(hand,qm);
grid on  

figure(2)
mySGplotHand(hand);
hold on
grid on 
 
hand = mySGaddFtipContact(hand,1,1:5);
 
[hand,object] = mySGmakeObject(hand); 
 
mySGplotObject(object);

delta_zr = [0 1 0 0]';
variation = mySGquasistatic(hand,object,delta_zr);

linMap = mySGquasistaticMaps(hand,object);

% object rigid body motions
rbmotion = mySGrbMotions(hand,object);

% find the optimal set of contact forces that minimizes SGVcost
E = ima(linMap.P);

ncont = size(E,2);

% contact properties
mu = 0.8;
alpha = 1/sqrt(1+mu^2);

% [K.Yao] Validation of force closure property
%{
P = hand.cp(1:3,:); % (3,N)
V = object.normals; % (3,N)
mu = 0.3;
k = 10;
ctr = object.center;
flag = forceClosureCheck(P, V, mu, k, ctr);
fprintf('Force closure grasp: %d\n', flag);
%}

fmin = 1;
%
fmax = 30;
%
k = 0.01;
% 
w = zeros(6,1);
%
pG = pinv(object.G);

y0 = rand(ncont,1);

% options.Display = 'iter';
option.TolX = 1e-3;
option.TolFun = 1e-3;
option.MaxIter = 5000;
option.MaxFunEvals = 500;

[yopt,cost_val] = fminsearch(@(y) mySGVcost(w,y,pG,E,object.normals,mu, fmin,fmax,k),y0,option);

lambdaopt = E*yopt;

c = mySGcheckFriction(w,yopt,pG,E,object.normals,alpha,k);

% solve the quasi static problem in the homogeneous form
Gamma = mySGquasistaticHsolution(hand, object);

% evlauate grasp stiffness matrix
Kgrasp = mySGgraspStiffness(hand,object);