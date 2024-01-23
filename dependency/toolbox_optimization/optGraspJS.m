% Optimal grasp planning in joint space.
% Objective function: grasp quality measure
% Optimization variable: joints

function [X_sol, fval, param, if_solution] = optGraspJS(hand, object, os, grasped_objects, if_plot_trial, if_save_trial)
% os_rmap: list of structures, contain only affected opposition spaces in desired grasp
% if_save: if save the optimization results
% grasped_objects: contains list of objects that have been grasped. these
% objects are used in InequalityConstraints (collision avoidance between target object and grasped objects)


if nargin < 6
    if_save_trial = false;
end

if nargin < 5
    if_plot_trial = false;
end    

if nargin < 4
    grasped_objects = {}; % empty as default
end

os_info = os.os_info;
os_rmap = os.os_rmap;
    
ncp = length(os_rmap); % number of contact points equals length of given opposition spaces

load('problem_config.mat');

qactv_loop = false(1,numel(hand.qactv)); % Used in this loop. create a list of joint angle activation status, used in optimization problem later
pactv = 0; % active flag of palm

tag = '';
ncpf = ncp; % number of contacts on fingers (exclude contact on palms)

for i = 1:ncp
    if ~all(os_info{i}) % palm is in the os list
        pactv = 1; % set palm as flag
        ncpf = ncpf - 1;
        tag(end+1) = 'P';
    else
        [idx_f,idx_l] = deal(os_info{i}(1),os_info{i}(2));
    
        nq_pre = min([idx_l, hand.F{idx_f}.n]); % number of joints ahead of idx_lnk, in case idx_lnk > finger.n (possible for fingertip link)
        q_idx = find(hand.qin == idx_f, 1); % first non-zero position (starting point index) of the indices of all joints of the finger in the hand
        
        h_qactv_states = hand.qactv(q_idx:q_idx+nq_pre-1); % real joint active states of current hand
        h_qactv_states = logical(h_qactv_states(:).');
        qactv_loop(q_idx:q_idx+nq_pre-1) = true(size(h_qactv_states)) & h_qactv_states; % set true, only if this joint is also active in the hand
        
        tag(end+1:end+4) = ['F',num2str(idx_f),'L',num2str(idx_l)]; % name tag of the link used for grasping
    end
    tag(end+1) = '_';
end

%% Initial States of Optimization Variables
xrange = zeros(2,ncp); % (lower, upper), boundaries of workspace rmaps, limits of alpha shape (approximated using rectangular)
yrange = zeros(2,ncp);
zrange = zeros(2,ncp);

for i = 1:ncp % iterate over contact points (variables)
    rmap_lb = min(os_rmap{i},[],1); % lower bound of rmap, should be (1,3), OS.os_rmap{i}: (n_pos,3)
    rmap_ub = max(os_rmap{i},[],1); % upper bound of rmap, should be (1,3)
    
    xrange(:,i) = [rmap_lb(1)-object.radius/2, rmap_ub(1)+object.radius/2];
    yrange(:,i) = [rmap_lb(2)-object.radius/2, rmap_ub(2)+object.radius/2];
    zrange(:,i) = [rmap_lb(3)-object.radius/2, rmap_ub(3)+object.radius/2];
end

lb_x = min(xrange(1,:)); % entire object must be constrained within the opposition space
lb_y = min(yrange(1,:));
lb_z = min(zrange(1,:));

ub_x = max(xrange(2,:));
ub_y = max(yrange(2,:));
ub_z = max(zrange(2,:));

objctr_0 = [(lb_x+ub_x)/2;...
    (lb_y+ub_y)/2;...
    (lb_z+ub_z)/2];

%%% joint angles
q_lb = hand.limit(:,1); % joint lower limit (fully open)
q_ub = hand.limit(:,2); % joint upper limit (fully closed)

r = 0.5; % use lower bound
q_0 = r*q_lb(qactv_loop) + (1-r)*q_ub(qactv_loop); % (nq_actv,1)

%%% cylindrical coordinates
phi_0 = zeros(ncpf,1); % in radius, angle of the cylindrical coordinate of contact point on the last link (end of the kinematic chain)
alp_0 = 0.5*ones(ncpf,1); % 'z' of the cylindrical coordinate at the last link (end of the kinematic chain)

%%% contact on the palm
pvec_lb = hand.P.pvec_lb(1:2);
pvec_ub = hand.P.pvec_ub(1:2);
pvec_0 = (pvec_ub+pvec_lb)/2; % palm translation vector (from palm center), on palm surface local CF (2,1)

%%% coefficients of force closure
if cstr.fc
    coeff_0 = ones(ncp*k,1)/(ncp*k); % friction cone coefficients, the friction cone on each contact point is approximated with k edges
else
    coeff_0 = [];
end

X0 = [objctr_0;... % (3,1), 1 : 3
    q_0(:);... % (nq_actv,1), 3+1 : 3+nq_actv
    phi_0(:);... % (ncpf,1), 3+nq_actv+1 : 3+nq_actv+ncpf
    alp_0(:);... % (ncpf,1), 3+nq_actv+ncpf+1 : 3+nq_actv+2*ncpf
    pvec_0(:);... % (2,1), 3+nq_actv+2*ncpf+1 : 3+nq_actv+2*ncpf+2
    coeff_0(:)]; % (k*ncp,1), 3+nq_actv+2*ncp+2+1 : 3+nq_actv+2*ncp+2+k*ncp

%%% Construct index for each part of the variables
nq_actv = sum(qactv_loop); % number of activated joints in this loop

idx_oc = false(size(X0));   idx_oc(1 : 3) = true;
idx_q = false(size(X0));    idx_q(3 +1 : 3 +nq_actv) = true;
idx_phi = false(size(X0));  idx_phi(3+nq_actv +1 : 3+nq_actv +ncpf) = true;
idx_alp = false(size(X0));  idx_alp(3+nq_actv+ncpf +1 : 3+nq_actv +2*ncpf) = true;
idx_pvec = false(size(X0)); idx_pvec(3+nq_actv+2*ncpf +1 : 3+nq_actv+2*ncpf +2) = true;

if cstr.fc % use force closure constraint
    idx_coeff = false(size(X0));
    idx_coeff(3+nq_actv+2*ncpf+2 +1 : 3+nq_actv+2*ncpf+2 +k*ncp) = true;
else
    idx_coeff = [];
end

%%% Construct key for variables
% dictionary of all variables
dict_q = sym('q%d%d',[hand.n, hand.m/hand.n]); % notice that matlab expands column-wise
dict_q = reshape(dict_q.',[],1);
dict_phi = sym('phi%d%d',[hand.n, hand.m/hand.n]);
dict_alp = sym('alp%d%d',[hand.n, hand.m/hand.n]);
if cstr.fc
    dict_coeff = sym('c%d%d',[ncp,k]);
    key_c = reshape(dict_coeff.',[],1);
else
    key_c = [];
end

% Keys of parameters
key_oc = [sym('x');sym('y');sym('z')];
key_q = dict_q(qactv_loop);
key_phi = sym(zeros(ncpf,1));
key_alp = sym(zeros(ncpf,1));
key_pvec = [sym('vx');sym('vy')]; % translation vector of palm on surface

for i = 1:ncp % filter out the idx of used phi and alp
    if ~all(os_info{i})
        continue; % palm contact does not need alp and phi
    else
        [idx_f,idx_l] = deal(os_info{i}(1),os_info{i}(2)); % index of finger that establishes contact and link in the finger
        key_phi(i) = dict_phi(idx_f, idx_l);
        key_alp(i) = dict_alp(idx_f, idx_l);
    end
end

X_key = [key_oc;...
    key_q(:);...
    key_phi(:);...
    key_alp(:);...
    key_pvec(:);...
    key_c];

fprintf('Optimization variable initialized. Number of variables: %d\n', length(X0));

%% Boundary Constraints
lb = zeros(size(X0));
ub = zeros(size(X0));

%%% obj_center, boundary of object centers
lb(1) = lb_x; % entire object must be constrained within the opposition space
lb(2) = lb_y;
lb(3) = lb_z;

ub(1) = ub_x;
ub(2) = ub_y;
ub(3) = ub_z;
% Only guarantee that object must lie inside two link rmaps, but no limit
% concerning the radius of object. E.g. distance between object center and
% the ub/lb must be at least larger than the radius of the object.

%%% q, boundary of joint angles % 20 in total
lb(idx_q) = q_lb(qactv_loop);
ub(idx_q) = q_ub(qactv_loop);

%%% boundary of phi (angle, [-pi~pi]) % 3 in total
lb(idx_phi) = -pi; % + deg2rad(1); % avoid overlap
ub(idx_phi) = pi;

%%% boundary of alp % 3 in total
lb(idx_alp) = 0;
ub(idx_alp) = 1;

%%% boundary of palm
lb(idx_pvec) = pvec_lb;
ub(idx_pvec) = pvec_ub;

%%% boundary of c % 15 in total
lb(idx_coeff) = 0;
ub(idx_coeff) = 1;

%% Linear Constraints
%%% Linear Inequality A*X <= b
A = [];
b = [];

%%% Linear Equality Aeq*X = beq
if cstr.fc % only exist for force closure constraints
    Aeq = zeros(1,length(X0));
    Aeq(idx_coeff) = 1; % sum over c_i is 1
    beq = 1;
else
    Aeq = [];
    beq = [];
end

%%% Input parameters for optimization problems
param.os = os; % saves list of (finger,link) information for opposition space
param.object_radius = object.radius; % radius of sphere object
param.hand_radius = hand.hand_radius; % i.e. 'link.radius'
param.grasped_objects = grasped_objects; % list of grasped objects

param.ncp = ncp; % number of contacts
param.k = k; % number of edges of friction cone
param.f_mu = f_mu; % coefficient of friction
param.f_gamma = f_gamma; % coefficient of torsinal friction

param.idx_oc = idx_oc; % index of object center in X
param.idx_q = idx_q; % index of active joint angles q in X 
param.idx_alp = idx_alp; % index of cylindrical coordinates alp in X
param.idx_phi = idx_phi; % index of cyl. coord phi in X
param.idx_coeff = idx_coeff; % index of coefficients of force closure in X
param.idx_pvec = idx_pvec;
param.X_key = X_key; % keys for identifying variables
param.dict_q = dict_q;

param.pactv = pactv; % palm active
param.qactv_loop = qactv_loop; % active joints being used in this optimization problem loop
param.cstr = cstr;

if ~all(X0>=lb)
    warning('X0 exceeds lower bounds.');
    X0(X0<=lb) = lb(X0<=lb);
end

if ~all(X0<=ub)
    warning('X0 exceeds upper bounds.');
    X0(X0>=ub) = ub(X0>=ub);
end

%% Nonlinear Constraints (Symbolic Expression)
[~,~,param] = symNonlInequalityConstraints(hand, param);
[~,~,param] = symNonlEqualityConstraints(hand, param);
nonlcon = @(X)optGraspJS_nonlcon(X);

%% Objective Function (Symbolic Expression)
symObjectiveFunction(hand, param);
objfun = @(X)optGraspJS_objfun(X);

%% Problem Configuration
options = optimoptions('fmincon',...
    'Algorithm','sqp',...% 'active-set', 'interior-point', 'sqp', 'trust-region-reflective', or 'sqp-legacy'
    'SpecifyObjectiveGradient',true,... % No gradient for objective function available
    'SpecifyConstraintGradient',true,...
    'Display','final',...% 'off', 'none', 'notify', 'notify-detailed', 'final', 'final-detailed', 'iter', or 'iter-detailed'
    'FunctionTolerance',tol_fun,...
    'MaxFunctionEvaluations',max_fun_evals,...
    'MaxIterations',max_iter,...
    'StepTolerance',tol_x);

disp('Starting fmincon...');

tic;
[X_sol,fval,exitflag,~] = fmincon(objfun, X0, A, b, Aeq, beq, lb, ub, nonlcon, options);
toc;

if exitflag<=0
    if_solution = false;
    tag(end+1:end+5) = 'false';
    fprintf('No feasible solution (exitflag: %d), checking results: \n', exitflag);
    evaluateOptimizationResults(X_sol, param); % Check if all constraints are satisfied
else
    if_solution = true;
    tag(end+1:end+4) = 'true';
    fprintf('Feasible solution found! (exitflag: %d)\n', exitflag);
end

if if_plot_trial
    visualizeOptimizationConfig(hand, 'default', X_sol, param, 'Trial result');
end

if if_save_trial
    save(['../database/results/optimization_results_',tag,'.mat'],...
        'hand','object',...
        'X_sol','fval','param');
    disp('* Optimization result saved.');
end

end