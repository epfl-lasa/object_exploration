% clc;
close all;
clear global; % removes all global variabels
warning off;

p = pwd;
if strcmp(p(end-2:end),'src') % in `src` folder
    addpath(genpath(fullfile(pwd,'..'))); % add paths to all subfolders in pwd
elseif strcmp(p(end-17:end),'inhand_exploration')
    addpath(genpath(pwd));
else
    error('Not in the package folder!');
end

rng(0,'twister'); % For reproducibility

%% DIRECTORY PARAMETERS
global path_results
% path_results = '/home/kunpeng/Dropbox/results';
% path_results = '/home/xiao/research/lasa/inhand_exploration/save_data';
% path_results = '/home/kunpeng/Workspace/inhand_exploration/src';
path_results = pwd;

global path_project
path_project = what('inhand_exploration').path;

% First level
global path_src path_database
path_src = fullfile(path_project, 'src');
path_database = fullfile(path_project, 'database');

% Second level
global path_shape_estimation path_hand_sampling path_symbolic
path_shape_estimation = fullfile(path_src, 'shape_estimation');
path_hand_sampling = fullfile(path_src, 'hand_sampling');
path_symbolic = fullfile(path_src, 'symbolic_functions'); % All symbolic functions and trained models saved here

global path_hand_model
path_hand_model = fullfile(path_database, 'hand_model'); % .../inhand_exploration/database/hand_model

%% PROBLEM CONFIGURATION PARAMETERS

%%% Configure problem parameters
global f_mu % coefficient of friction
f_mu = 0.5;

global f_gamma % coefficient of torsional friction
f_gamma = 1.0;

global fc_k % number of edges for approximating the friction cone
fc_k = 3;

% Approximated friction cone of finger contacts (contact normal: +Y direction)
global fc_S
fc_S = [f_mu*cos(2*pi*(1:fc_k)/fc_k);... % approximation of friction cone
    ones(1,fc_k);... % central axis of cone: pointing in the y+ direction
    f_mu*sin(2*pi*(1:fc_k)/fc_k)];

% Approximated friction cone of palm contacts (contact normal: -Z direction)
global fc_Sp
fc_Sp = [f_mu*cos(2*pi*(1:fc_k)/fc_k);... % approximation of friction cone
    f_mu*sin(2*pi*(1:fc_k)/fc_k);...
    -ones(1,fc_k)]; % this cone is the torsional torque, so the central axis, Y, of the S, corresponds to a central axis in -Z, of Sp
  
global epsilon % numerical round-off error tolerance
epsilon = 1e-6;

global ApproxCylNum % minimum sample number along cylinder axis (in collision avoidance)
ApproxCylNum = 5;

global Hz
Hz = 10; % sampling frequency (1/Hz is the time spent to move to the next point)

global timeSteps
timeSteps = 5; % For visualization
