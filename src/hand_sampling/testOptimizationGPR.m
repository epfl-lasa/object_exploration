% This script tests the performance in constrained optimization, where a
% GPR models is used as objective function. The aim is to search for the
% points that has the largest value.
configuration;

% iF = 1;
iF = 2;

testType = 'isotropy';
% testType = 'omega';

switch iF
    case 1 % Thumb
        data = load('F1L5_FeatureModel.mat'); % Load raw training data
        if strcmp(testType, 'omega')
            gprMdl = load('oMT.mat');
            gprMdl = gprMdl.oMT;
            datay = data.omegaMap; % (N,1)
        elseif strcmp(testType, 'isotropy')
            gprMdl = load('iMT.mat');
            gprMdl = gprMdl.iMT;
            datay = data.isoMap; % (N,1)
        end
    otherwise % Fingers
        data = load('F2L5_FeatureModel.mat');
        if strcmp(testType, 'omega')
            gprMdl = load('oMF.mat');
            gprMdl = gprMdl.oMF;
            datay = data.omegaMap;
        elseif strcmp(testType, 'isotropy')
            gprMdl = load('iMF.mat');
            gprMdl = gprMdl.iMF;
            datay = data.isoMap; % (N,1)
        end
end

dataX = data.jointAngles; % (N,4)
rMap = data.rMap; % (N,3) for plotting Cartesian space points

% Create optimization variables
q = optimvar('q',[1,4],'Type','continuous','LowerBound',min(dataX,[],1),'UpperBound',max(dataX,[],1)); % (1,4)
q0 = q.LowerBound + rand*(q.UpperBound - q.LowerBound);

%% Load hand model (for evaluation)
hand_file = load(fullfile(path_hand_model, 'AllegroHandLeft_model.mat'));
hand = hand_file.hand;
symFK = hand.F{iF}.Link{end}.symbolic.HT_next(1:3,4);
Q = sym(strcat('q',num2str(iF),'%d'), [1,4]);

%% Constraints
A = [];
b = [];
Aeq = [];
beq = [];

nonlcon = [];
fun = @(X)objectiveFcn(X);

options = optimoptions('fmincon','Display','final','Algorithm','sqp');

timeElapsed = zeros(1,10);
for i = 1:10
    tic;
    [X_sol,fval,exitflag,output] = fmincon(fun, q0, A, b, Aeq, beq, q.LowerBound, q.UpperBound, nonlcon, options);

    % [X_sol,fval,eflagfms,outputfms] = fminsearch(fun, q0);

    % [X_sol,fval,eflagps,outputps] = patternsearch(fun, q0);
    timeElapsed(i) = toc;
end
fprintf('Average time spent: %d +/- %d\n\', mean(timeElapsed), std(timeElapsed)); % 2.242421e-01 +/- 3.774016e-02

%% Evaluate solution
[max_omega, idx_omega] = max(datay);

fprintf('Max. objective value in training data: %d\n', max_omega);
disp('Corresponding opt. variables: ');
disp(dataX(idx_omega, :));

fprintf('Max. objective value obtained by solver: %d\n', 1./fval);
disp('Solution joint angles: ');
disp(X_sol);

figure, hold on;
scatter3(rMap(:,1),rMap(:,2),rMap(:,3),20,datay);
hold on;

numFK = double(subs(symFK, Q, X_sol));
scatter3(numFK(1),numFK(2),numFK(3),200,fval,'filled');
scatter3(rMap(idx_omega,1),rMap(idx_omega,2),rMap(idx_omega,3),200,max_omega,'filled');

grid on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Comparison of optimization solution and real solution');
hold off;

%% Objective function
function y = objectiveFcn(X)

%     y = predict(gprMdl,X);
%     y = 1./y; % to maximize omega index
%     y = omegaValueThumb(X);

%     y = 1./isotropyValueThumb(X);
    y = 1./isotropyValueFinger(X);
end

% Evaluate GPR model (temporary solution) % Use: `evaluationTrainedMetricModels.m`