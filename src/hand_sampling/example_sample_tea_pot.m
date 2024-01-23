% This script simulates the reconstruction process of object shape based on
% tactile inputs (contact points).
% K. Yao, Dec. 2020

close all;
clear all;
clc;

%% Object geometric Model
%{
addpath(genpath([pwd, '\..\..\hand_dexterity\SynGrasp'])); % add path to SynGrasp pkg

ctr = [0;0;0];
Htr = [rotz(30)*roty(45),ctr;...
    0,0,0,1];
a = 7; % length
b = 11; % width
c = 13; % height

Cube = SGcube(Htr,a,b,c);
SGplotObject(Cube);
view([45,45]);
%}

%% Object point cloud model
ptCloud = pcread('teapot.ply'); % tea pot
pcshow(ptCloud);

%%% Surface normal of point cloud data
%{
normals = pcnormals(ptCloud);
x = ptCloud.Location(1:10:end,1);
y = ptCloud.Location(1:10:end,2);
z = ptCloud.Location(1:10:end,3);
u = normals(1:10:end,1);
v = normals(1:10:end,2);
w = normals(1:10:end,3);
quiver3(x,y,z,u,v,w);
%}

%%% Assumption: contacts are made on a plane parallel to the Z-plane
z_min = ptCloud.ZLimits(1); % object upper bound
z_max = ptCloud.ZLimits(2); % object lower bound
precision = (z_max-z_min)/100; % tolerance range of contact points

%% Sampling process: In robotic hand reference frame
config.rot_mu = 5; % average rotation angles
config.rot_std = 1; % rotation standard deviation
config.num_trials = 20; % number of rotation steps
config.num_fingers = 4; % number of fingers
config.precision = precision;
config.if_plot = false; % plot result
config.ws_margin = 0.1; % define margin of workspace
config.noise_sigma = 0.01; % noise at sampling point coordination, sigma in normal distribution

config.step_length = precision/3; % step length of finger motion

completeSampleSet = [];

num_height = 10;
for h = 1:num_height
    explore_height = z_min + rand * (z_max-z_min); % determine a height to explore [the plane of contact points]
    heightSampleSet = sampleHeight(ptCloud, explore_height, config);
    
    completeSampleSet = cat(1, completeSampleSet, heightSampleSet); % (:,3)
end

figure, hold on;
scatter3(completeSampleSet(:,1),completeSampleSet(:,2),completeSampleSet(:,3),'filled');
axis equal;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
disp('Completed.');