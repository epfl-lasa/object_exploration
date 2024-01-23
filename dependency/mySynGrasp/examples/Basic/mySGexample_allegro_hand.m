%    SGexample_allegro_hand - Basic example to use the Allegro Hand model
%
%    Usage: SGexample_allegro_hand
%
%  This file is part of SynGrasp (Synergy Grasping Toolbox).
%
%  All rights reserved.
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

close all;
clear all;
clc;

% warning off;

% Hand definition 
T_hand_ini = [SGrotz(pi/2)*SGrotx(pi/2)*SGroty(pi/2) [0 0 0]';
    0 0 0 1];

% hand = mySGallegroLeft(T_hand_ini);
hand = mySGallegroRight(T_hand_ini);

mySGplotHand(hand);

% Home position
q_home = [0.7299820072825081, 0.4579193740155175, 0.23919718596737496, 0.7686472393976554, ...
    -0.02785654223134834, 0.009314805919159919, 0.918914391495951, 0.7941311423884578, ...
    0.0066785400981930945, -0.005536158235084111, 0.8715494821394858, 0.764692840660485, ...
    0.06300675323675811, 0.029174675142167622, 0.868913216319781, 0.8087184799534589];

% Move to home position
hand = mySGmoveHand(hand, q_home');

% Plot
figure;
mySGplotHand(hand);
hold on;
xlabel('x');
ylabel('y');
zlabel('z');