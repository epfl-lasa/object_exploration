%% Configure directories
clc;
clear all;
close all;

package_path = what('inhand_exploration');
if isempty(package_path.path) % path to package ‘inhand_exploration’
    warning('Current package cannot be found: inhand_exploration');
    package_path = fullfile(pwd,'..','..');
    addpath(genpath(package_path));
end
package_path = package_path.path;
addpath(genpath(package_path));

%%% Level 0 directories
src_dir = fullfile(package_path, 'src'); % to 'src'
database_dir = fullfile(package_path, 'database'); % to database
toolbox_dir = fullfile(package_path, 'toolbox');
symbolic_functions_dir = fullfile(database_dir, 'symbolic_functions');

addpath(genpath(symbolic_functions_dir));

%%% Level 1 directories
hand_sampling_dir = fullfile(src_dir, 'hand_sampling');
shape_estimation_dir = fullfile(src_dir, 'shape_estimation');

%% Configure parameters
k = 3; % number of edges for approximating the friction cone
f_mu = 0.5; % coefficient of friction
f_gamma = 1.0; % coefficient of torsinal friction

% Approximated friction cone of finger contacts (contact normal: +Y direction)
S = [f_mu*cos(2*pi*[1:k]/k);... % approximation of friction cone
    ones(1,k);... % central axis of cone: pointing in the y+ direction
    f_mu*sin(2*pi*[1:k]/k)];

% Approximated friction cone of palm contacts (contact normal: -Z direction)
Sp = [f_mu*cos(2*pi*[1:k]/k);... % approximation of friction cone
    f_mu*sin(2*pi*[1:k]/k);...
    -ones(1,k)]; % this cone is the torsinal torque, so the central axis, Y, of the S, corresponds to a central axis in -Z, of Sp

epsilon = 1e-6; % numerical round-off error tolerance

% Config for optimization
max_fun_evals = 100000;
max_iter = 5000;
tol_fun = 1e-4;
tol_x = 1e-6;

% Save cofiguration
save(fullfile(src_dir, '/problem_config.mat'));
disp('File saved: problem_config.mat');