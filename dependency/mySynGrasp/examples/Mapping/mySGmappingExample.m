%    SGmappingExample - SynGrasp demo concerning the mapping of
%    human hand synergies onto the modular hand model
%    
%    Usage: SGmappingExample
%
%    References:
%    1. G. Gioioso, G. Salvietti, M. Malvezzi, D. Prattichizzo. Mapping 
%    Synergies from Human to Robotic Hands with Dissimilar Kinematics: an 
%    Approach in the Object Domain. IEEE Trans. on Robotics, 2013.
%    2. G. Salvietti, L. Meli, G. Gioioso, M. Malvezzi, D. Prattichizzo. 
%    Multi-Contact Bilateral Telemanipulation with Kinematic Asymmetries. 
%    IEEE/ASME Transaction on Mechatronics, 2017.
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

clear all
close all
clc

%% HUMAN HAND initialization
hand = mySGparadigmatic;


[qm, S] = mySGsantelloSynergies;
 
hand = mySGdefineSynergies(hand,S,qm); 

hand = mySGmoveHand(hand,qm);

hand = mySGaddFtipContact(hand,1,1:5);

[hand,object] = mySGmakeObject(hand);

xyz = mySGfingertips(hand);
[csph,rsph] = minboundsphere(xyz');
    
A = mySGmapping_A(hand,csph);

%% ROBOTIC HAND initialization

hand_r = mySGmodularHand;

% defining starting position
q1 = [pi/4 pi/3 pi/3];
q2 = [pi/4 pi/3 pi/3];
q3 = [pi/4 pi/3 pi/3];

q_r = [q1,q2,q3];
q_r_init = q_r;

Q = q_r_init;

hand_r = mySGmoveHand(hand_r,q_r);


hand_r = mySGaddFtipContact(hand_r,1,1:3);

[hand_r,object_r] = mySGmakeObject(hand_r);

xyz_r = mySGfingertips(hand_r);
[csph_r,rsph_r] = minboundsphere(xyz_r');
    
A_r = mySGmapping_A(hand_r,csph_r);

k_mapping = rsph_r / rsph;

Sc = mySGmapping_SC(k_mapping);

S_r = pinv(hand_r.J) * A_r * Sc * pinv(A) * hand.J * hand.S;
hand_r = mySGdefineSynergies(hand_r,S_r,hand_r.q);


%% Graphics: positioning figures

set(0,'Units','pixels') 
scnsize = get(0,'ScreenSize');
fig1 = figure(1);


fig2 = figure(2);

position = get(fig1,'Position');
outerpos = get(fig1,'OuterPosition');
borders = outerpos - position;

edge = -borders(1)/2;
pos1 = [edge,...
        edge,...
        scnsize(3)/2,...
        scnsize(4)+edge];
pos2 = [scnsize(3)/2 + edge,...
        pos1(2),...
        pos1(3),...
        pos1(4)];
    
set(fig1,'OuterPosition',pos1); 
set(fig2,'OuterPosition',pos2);

% plotting hands
figure(1)
mySGplotHand(hand);
axis on
grid on
view([-154 12]);
axis([-70 50 20 150 -80 0]);

figure(2)
mySGplotHand(hand_r);
axis on
grid on 
view([-144 32]);
xlabel('x');
ylabel('y');
zlabel('z');
axis([-100 80 -20 100 -100 0]);


%% START MAPPING

% simulation parameters

simulation_time = 4;
sampling_time = 0.1;
samples = simulation_time / sampling_time;



while(1)
    
    syn_index = input('Select a synergy (1:3) or press 0 to exit: ');

    switch syn_index
        case 0 
            break;
        case 1 
            zpunto = [-1 0 0 0 0 0 0 0 0 0 0 0 0 0 0]';
        
        case 2 
            zpunto = [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0]';
        
        case 3
            zpunto = [0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0]';
            
    end
    
Q = [];
delta_z = zpunto*sampling_time;

for i=1:samples/4
    
    
    
    %% human hand update
    
    hand = mySGactivateSynergies(hand,delta_z);
    ppunto = hand.J * hand.S*zpunto;
    
    parameters = pinv(A) * ppunto;
    
    csph = csph + sampling_time*parameters(1:3)';

    A = mySGmapping_A(hand,csph);
 
    figure(1)
    mySGplotHand(hand);
    axis on
    grid on
    view([-154 12]);
    axis([-70 50 20 150 -80 0]);

    
    %% robotic hand
    
    hand_r = mySGactivateSynergies(hand_r,delta_z);
    ppunto_r = hand_r.J * hand_r.S*zpunto;
    
    
    Q = [Q; hand_r.q'];
    parameters_r = pinv(A_r) * ppunto_r;
    
    csph_r = csph_r + sampling_time*parameters_r(1:3)';
        
    A_r = mySGmapping_A(hand_r,csph_r);
    
    
    S_r = pinv(hand_r.J) * A_r * Sc * pinv(A) * hand.J * hand.S;
    hand_r = mySGdefineSynergies(hand_r,S_r,hand_r.q);
     
end

zpunto = -zpunto;
delta_z = zpunto*sampling_time;

for i=1:samples/2
    
    
    
    %% human hand update

    hand = mySGactivateSynergies(hand,delta_z);
    ppunto = hand.J * hand.S*zpunto;
    
    parameters = pinv(A) * ppunto;
    
    csph = csph + sampling_time*parameters(1:3)';
    
    A = mySGmapping_A(hand,csph);
   
  
    figure(1)
    mySGplotHand(hand);
    axis on
    grid on 
    view([-154 12]);
    axis([-70 50 20 150 -80 0]);
    

    
    %% robotic hand
    
    hand_r = mySGactivateSynergies(hand_r,delta_z);
    ppunto_r = hand_r.J * hand_r.S*zpunto;
    
    
    Q = [Q; hand_r.q'];
    parameters_r = pinv(A_r) * ppunto_r;
    
    csph_r = csph_r + sampling_time*parameters_r(1:3)';
    
    A_r = mySGmapping_A(hand_r,csph_r);
    
    
    S_r = pinv(hand_r.J) * A_r * Sc * pinv(A) * hand.J * hand.S;
    hand_r = mySGdefineSynergies(hand_r,S_r,hand_r.q);

    
end

zpunto = -zpunto;
delta_z = zpunto*sampling_time;

for i=1:samples/4
    
    
    
    %% human hand update
    hand = mySGactivateSynergies(hand,delta_z);
    ppunto = hand.J * hand.S*zpunto;
        

    
    parameters = pinv(A) * ppunto;
    
    csph = csph + sampling_time*parameters(1:3)';
    
    A = mySGmapping_A(hand,csph);
   
 
    figure(1)
    mySGplotHand(hand);
    axis on
    grid on 
    view([-154 12]);
    axis([-70 50 20 150 -80 0]);
    

    
    %% robotic hand
    
    hand_r = mySGactivateSynergies(hand_r,delta_z);
    ppunto_r = hand_r.J * hand_r.S*zpunto;
    
    
    Q = [Q; hand_r.q'];
    parameters_r = pinv(A_r) * ppunto_r;
    
    csph_r = csph_r + sampling_time*parameters_r(1:3)';
    
    A_r = mySGmapping_A(hand_r,csph_r);
    
    
    S_r = pinv(hand_r.J) * A_r * Sc * pinv(A) * hand.J * hand.S;
    hand_r = SGdefineSynergies(hand_r,S_r,hand_r.q);

    
end


for i=1:samples

    q_r = Q(i,:);
    hand_r = mySGmoveHand(hand_r,q_r);
    
    figure(2)
    mySGplotHand(hand_r);
    axis on
    grid on 
    view([-144 32]);
    axis([-100 80 -20 100 -100 0]);
    
end

end