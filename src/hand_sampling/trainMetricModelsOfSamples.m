% This file trains the mapping btw Allegro hand finger joint angles (thumb
% or finger) and metric values (manipulability index, 'omegaValue', or isotropy
% value of the reachability map, 'isotropyValue') using GPR, and save them
% as local files (both matlabFunction and .mat files). Such files will be
% used in the optimization framework 'main_exploration' for online
% evaluation.

configuration;
warning off;
close all;

% Customization: number of data points used for training
nTrain = 1000;

global path_symbolic
assert(~isempty(path_symbolic));

if ~exist(path_symbolic, 'dir')
    mkdir(path_symbolic);
end

dataT = load('F1L5_FeatureModel.mat'); % Samples from Allegro hand model, has fields: 'rMap','omegaMap','isoMap','jointAngles'
dataF = load('F2L5_FeatureModel.mat'); % Allegro hand has same models for Finger 2, 3, and 4

%% GPR
N = size(dataT.jointAngles, 1); % Original number of samples: 10000
assert(N==10000);

% ds = 10; % downsample factor, GPR is slow for original N value (10k)
% ds = 1;
% ds = ceil(N/nTrain);

% idx = 1:ds:N; % uniformly sample leads to degenerated dataset
% idx = randi([1 N],1,N/ds);
idx = randi([1 N],1,nTrain); % randomly select points as training data

X = sym('x',[1,4]); % Symbolic variables: [x1,x2,x3,x4]
extra_test = true; % if generate new data for testing model

visualizeMap(dataT.rMap(idx,:), dataT.omegaMap(idx,:), 'Sampled Manipulability Index of Thumb');
visualizeMap(dataT.rMap(idx,:), dataT.isoMap(idx,:),   'Sampled Isotropic Reachability Index of Thumb');
visualizeMap(dataF.rMap(idx,:), dataF.omegaMap(idx,:), 'Sampled Manipulability Index of Finger');
visualizeMap(dataF.rMap(idx,:), dataF.isoMap(idx,:),   'Sampled Isotropic Reachability Index of Finger');

%% Train GPR for each finger type (Only two types for AllegroHand: Thumb or Finger)
%%% (1) Thumb Models (dataT)
jointAngles = dataT.jointAngles(idx,:);
omegaMap = dataT.omegaMap(idx,:);
isoMap = dataT.isoMap(idx,:);
rMap = dataT.rMap(idx,:);

% Normalization of values
omegaMap = omegaMap./max(omegaMap);
isoMap = isoMap./max(isoMap);

[oMT, omegaSymbolicThumb] = trainGPRModel(jointAngles, omegaMap); % (10000,4), (10000,1)
% save(fullfile(path_symbolic,'omegaSymbolicThumb.mat'),'omegaSymbolicThumb'); % Save the expression of desired metric (i.e., 'y')
% save(fullfile(path_symbolic,'oMT.mat'),'oMT'); % save the fit GPR model
% matlabFunction(omegaSymbolicThumb,'File',fullfile(path_symbolic,'omegaSymbolicThumb.m'),'Vars',{X},'Optimize',false);
% evaluateTrainedMetricModels(oMT, 'oMT', jointAngles, omegaMap, rMap, extra_test, 1);

[iMT, isotropySymbolicThumb] = trainGPRModel(jointAngles, isoMap);
% save(fullfile(path_symbolic,'isotropySymbolicThumb.mat'),'isotropySymbolicThumb');
% save(fullfile(path_symbolic,'iMT.mat'),'iMT');
% matlabFunction(isotropySymbolicThumb,'File',fullfile(path_symbolic,'isotropySymbolicThumb.m'),'Vars',{X},'Optimize',false);
% evaluateTrainedMetricModels(iMT, 'iMT', jointAngles, isoMap, rMap, extra_test, 1);

%%% (2) Finger Models (dataF)
jointAngles = dataF.jointAngles(idx,:);
omegaMap = dataF.omegaMap(idx,:);
isoMap = dataF.isoMap(idx,:);
rMap = dataF.rMap(idx,:);

% Normalization of values
omegaMap = omegaMap./max(omegaMap);
isoMap = isoMap./max(isoMap);

[oMF, omegaSymbolicFinger] = trainGPRModel(jointAngles, omegaMap);
% save(fullfile(path_symbolic,'omegaSymbolicFinger.mat'),'omegaSymbolicFinger');
% save(fullfile(path_symbolic,'oMF.mat'),'oMF');
% matlabFunction(omegaSymbolicFinger,'File',fullfile(path_symbolic,'omegaSymbolicFinger.m'),'Vars',{X},'Optimize',false);
% evaluateTrainedMetricModels(oMF, 'oMF', jointAngles, omegaMap, rMap, extra_test, 2);

[iMF, isotropySymbolicFinger] = trainGPRModel(jointAngles, isoMap);
% save(fullfile(path_symbolic,'isotropySymbolicFinger.mat'),'isotropySymbolicFinger');
% save(fullfile(path_symbolic,'iMF.mat'),'iMF');
% matlabFunction(isotropySymbolicFinger,'File',fullfile(path_symbolic,'isotropySymbolicFinger.m'),'Vars',{X},'Optimize',false);
% evaluateTrainedMetricModels(iMF, 'iMF', jointAngles, isoMap, rMap, extra_test, 2);

%% Generate parameter-specified local matlab files for each finger type
fprintf('Create specified models for each finger...\n');

sym_q = sym('q%d%d',[4,4]); % (1,16), joint angles

%% Manipulability models (finger-specified)
omegaFunctionThumb = subs(omegaSymbolicThumb,  X, sym_q(1,:)); % Substitude models with parameters from each fingers individually
omegaFunctionIndex = subs(omegaSymbolicFinger, X, sym_q(2,:));
omegaFunctionMiddle= subs(omegaSymbolicFinger, X, sym_q(3,:));
omegaFunctionRing  = subs(omegaSymbolicFinger, X, sym_q(4,:));

assert(isequal(symvar(omegaFunctionThumb), sym_q(1,:)));
assert(isequal(symvar(omegaFunctionIndex), sym_q(2,:)));
assert(isequal(symvar(omegaFunctionMiddle),sym_q(3,:)));
assert(isequal(symvar(omegaFunctionRing),  sym_q(4,:)));

%%%% Format I: save as MATLAB functions
matlabFunction(omegaFunctionThumb, 'File',fullfile(path_symbolic,'omegaFunctionThumb.m'), 'Vars',symvar(omegaFunctionThumb), 'Optimize',false);
matlabFunction(omegaFunctionIndex, 'File',fullfile(path_symbolic,'omegaFunctionIndex.m'), 'Vars',symvar(omegaFunctionIndex), 'Optimize',false);
matlabFunction(omegaFunctionMiddle,'File',fullfile(path_symbolic,'omegaFunctionMiddle.m'),'Vars',symvar(omegaFunctionMiddle),'Optimize',false);
matlabFunction(omegaFunctionRing,  'File',fullfile(path_symbolic,'omegaFunctionRing.m'),  'Vars',symvar(omegaFunctionRing),  'Optimize',false);

%%%% Format II: save as MATLAB models
save(fullfile(path_symbolic,'omegaFunctionThumb.mat'), 'omegaFunctionThumb');
save(fullfile(path_symbolic,'omegaFunctionIndex.mat'), 'omegaFunctionIndex');
save(fullfile(path_symbolic,'omegaFunctionMiddle.mat'),'omegaFunctionMiddle');
save(fullfile(path_symbolic,'omegaFunctionRing.mat'),  'omegaFunctionRing');

%% Isotropy functions (finger-specified)
isotropyFunctionThumb = subs(isotropySymbolicThumb,  X, sym_q(1,:));
isotropyFunctionIndex = subs(isotropySymbolicFinger, X, sym_q(2,:));
isotropyFunctionMiddle= subs(isotropySymbolicFinger, X, sym_q(3,:));
isotropyFunctionRing  = subs(isotropySymbolicFinger, X, sym_q(4,:));

assert(isequal(symvar(isotropyFunctionThumb), sym_q(1,:)));
assert(isequal(symvar(isotropyFunctionIndex), sym_q(2,:)));
assert(isequal(symvar(isotropyFunctionMiddle),sym_q(3,:)));
assert(isequal(symvar(isotropyFunctionRing),  sym_q(4,:)));

%%%% Format I: save as MATLAB functions
save(fullfile(path_symbolic,'isotropyFunctionThumb.mat'), 'isotropyFunctionThumb');
save(fullfile(path_symbolic,'isotropyFunctionIndex.mat'), 'isotropyFunctionIndex');
save(fullfile(path_symbolic,'isotropyFunctionMiddle.mat'),'isotropyFunctionMiddle');
save(fullfile(path_symbolic,'isotropyFunctionRing.mat'),  'isotropyFunctionRing');

%%%% Format II: save as MATLAB models
matlabFunction(isotropyFunctionThumb, 'File',fullfile(path_symbolic,'isotropyFunctionThumb.m'), 'Vars',symvar(isotropyFunctionThumb), 'Optimize',false);
matlabFunction(isotropyFunctionIndex, 'File',fullfile(path_symbolic,'isotropyFunctionIndex.m'), 'Vars',symvar(isotropyFunctionIndex), 'Optimize',false);
matlabFunction(isotropyFunctionMiddle,'File',fullfile(path_symbolic,'isotropyFunctionMiddle.m'),'Vars',symvar(isotropyFunctionMiddle),'Optimize',false);
matlabFunction(isotropyFunctionRing,  'File',fullfile(path_symbolic,'isotropyFunctionRing.m'),  'Vars',symvar(isotropyFunctionRing),  'Optimize',false);

addpath(path_symbolic); % Add all saved function files to path for retreaving
disp('Training metric model of samples finished.');

%{
%% Evaluate time performance

fprintf('Evaluating the computational time cost for trained model...\n');
nTest = 100;

clear omegaValueThumb; % clear local variables in the workspace to make sure the saved matlab function file is called
clear isotropyValueThumb;
clear omegaValueFinger;
clear isotropyValueFinger;

oMT_t = zeros(1,nTest);
iMT_t = zeros(1,nTest);
oMF_t = zeros(1,nTest);
iMF_t = zeros(1,nTest);

for i = 1:nTest
    tic;
    omegaValueThumb(rand(1,4));
    oMT_t(i) = toc;
    
    tic;
    isotropyValueThumb(rand(1,4));
    iMT_t(i) = toc;
    
    tic;
    omegaValueFinger(rand(1,4));
    oMF_t(i) = toc;
    
    tic;
    isotropyValueFinger(rand(1,4));
    iMF_t(i) = toc;
end

fprintf('Average time of evaluation omegaValueThumb: %d, std: %d\n', mean(oMT_t), std(oMT_t));
fprintf('Average time of evaluation isotropyValueThumb: %d, std: %d\n', mean(iMT_t), std(iMT_t));
fprintf('Average time of evaluation omegaValueFinger: %d, std: %d\n', mean(oMF_t), std(oMF_t));
fprintf('Average time of evaluation isotropyValueFinger: %d, std: %d\n', mean(iMF_t), std(iMF_t));
%}




function visualizeMap(rMap, valueMap, figure_title)
    figure, hold on;
    scatter3(rMap(:,1),rMap(:,2),rMap(:,3),30,valueMap,'filled');
    colorbar;
    axis equal;
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
%     title(figure_title);
    view([-30, 30]);
    hold off;
    figure_title = strrep(figure_title,' ','_');
    saveas(gcf,strcat(figure_title,'.png'));
    close(gcf);
end


%% [deprecated] GMM
%{
oMT = trainGMModel(dataT.jointAngles, dataT.omegaMap); % Train models for thumb
evaluationModels(dataT, oMT, 'omega');
% y_oMT = predictGMModel(oMT,X); % obtain symbolic expression of prediction
% matlabFunction(y_oMT,'File','oMTValue','Vars',{X},'Optimize',false);

oMF = trainGMModel(dataF.jointAngles, dataF.omegaMap); % Train models for fingers
evaluationModels(dataF, oMF, 'omega');
% y_oMF = predictGMModel(oMF,X);
% matlabFunction(y_oMF,'File','oMFValue','Vars',{X},'Optimize',false);

iMT = trainGMModel(dataT.jointAngles, dataT.isoMap); % iMT: isotropy Model of Thumb
evaluationModels(dataT, iMT, 'isotropy');
% y_iMT = predictGMModel(iMT,X);
% matlabFunction(y_iMT,'File','iMTValue','Vars',{X},'Optimize',false);

iMF = trainGMModel(dataF.jointAngles, dataF.isoMap); % iMF: isotropy Model of Finger
evaluationModels(dataF, iMF, 'isotropy');
% y_iMF = predictGMModel(iMF,X);
% matlabFunction(y_iMF,'File','iMFValue','Vars',{X},'Optimize',false);

disp('All symbolic expression of Gaussian Mixture Model predictions are saved.');
%}