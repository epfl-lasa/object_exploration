% Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
% 
% This file is originally part of the SynGrasp package and has been modified by
% Kunpeng Yao (kunpeng.yao@epfl.ch), LASA, EPFL.
% 
% This file is for personal use only, and you are not allowed to publish it.
% For free public software, please use the original SynGrasp package instead.

function hand = mySGparadigmatic(T)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    1 - P A R A D I G M A T I C   H A N D    P A R A M E T E R S
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if nargin<1
    T = eye(4); % homo transform from the origin of the world to the base of the hand
end

hand_type = 'mySGparadigmatic'; % THIS IS A RIGHT HAND

FinTipDim = 10; % finger tip diameter

%%% Pre-allocation
DHpars{5} = [];
base{5} = [];
F{5} = []; % fingers struct

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Thumb
% [K. Yao] Comments: definition of DHpars mtx:
% * DHpars(i,:): DH-params of the ith link
% * DHpars(:,3): rotation angles
% * order of parameters in each row:
%   * alpha: the angle between z_i-1 and z_i rotated around x_i
%   * a: distance along x_i from the intersection to o_i
%   * theta: the angle between x_i-1 and x_i rotated around z_i-1 (axis of motion)
%   * d: distance along z_i-1 from o_i-1 to the intersection of the x_i and z_i-1 axes
%
% Remark: Check another hand model defined in 'SAHR/skeleton_model/src/hand_model' for a 5 DoF hand model


%%% Modified parameters by K. Yao
% Links' lengths (only thumb)
Ltm = 25;
Ltp = 15;
Ltd = 10;

DHpars{1} = [
    -pi/2 0 0 0; % alpha, a, theta, d
       0 Ltm 0 0;
       -pi/2 Ltp 0 0; % pi/2 Ltp 0 0;
       0 Ltd 0 0];

base{1} = [0 -1 0 -37;
    1 0 0 45;
    0 0 1 0;
    0 0 0 1];

%{
%%% Original parameters defined in SynGrasp mySGparadigmatic hand
DHpars{1} = [
    -pi/2 0 0 0;
       0 25 0 0;
    pi/2 15 0 0;
       0 10 0 0];

base{1} = [0 -1 0 -37;
    1 0 0 45;
    0 0 1 0
    0 0 0 1];
%}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Index
DHpars{2} = [
    -pi/2 0 0 0;        % MCP joint (abduction/adduction)
       0 37 0 0;        % MCP joint (flexion/extention)
       0 30 0 0;        % PIP joint (flexion/extention)
       0 15 0 0];       % DIP joint (flexion/extention)
 
base{2} = [0 -1 0 -24;
    1 0 0 73;
    0 0 1 0;
    0 0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Middle
DHpars{3} = [
    -pi/2 0 0 0;        % MCP joint (abduction/adduction)
       0 40 0 0;        % MCP joint (flexion/extention)
       0 35 0 0;        % PIP joint (flexion/extention)
       0 17 0 0];       % DIP joint (flexion/extention)

base{3} = [0 -1 0 -8;
    1 0 0 73;
    0 0 1 0;
    0 0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%% Ring
DHpars{4} = [
    -pi/2 0 0 0;        % MCP joint (abduction/adduction)
       0 37 0 0;        % MCP joint (flexion/extention)
       0 30 0 0;        % PIP joint (flexion/extention)
       0 15 0 0];       % DIP joint (flexion/extention)

base{4} = [0 -1 0 8;
    1 0 0 73;
    0 0 1 0;
    0 0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%% Little
DHpars{5} = [
    -pi/2 0 0 0;        % MCP joint (abduction/adduction)
       0 27 0 0;        % MCP joint (flexion/extention)
       0 25 0 0;        % PIP joint (flexion/extention)
       0 10 0 0];       % DIP joint (flexion/extention)

base{5} = [0 -1 0 24;
    1 0 0 73;
    0 0 1 0;
    0 0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

cc = struct('rho',FinTipDim/2,'phi',0,'alp',0.5); % default values of cylindrical coordinates. cc is used to calculate contact points in link.

for i = 1:length(DHpars)
    n = size(DHpars{i},1); % number of joints for each finger, each row is a dof
    q = zeros(n,1); % initialize joint variables
    
    % Change default joint bounds for each joint
    if i == 1 % thumb
        lb = deg2rad([-60, 0, 0, 0]); % lower bound
        % lb = deg2rad([0, 0, 0, 0]);
        ub = deg2rad([60, 60, 90, 90]); % upper bound
    else
        lb = deg2rad([-15, 0, 0, 0]); % lower bound
        ub = deg2rad([15, 90, 90, 90]); % upper bound
    end
    
    F{i} = mySGmakeFinger(DHpars{i},T*base{i},q,i,...
        lb,ub,...% lb, ub
        'sym',[],[],...% eval_type (calc symbolic), active_joints, contacted_link
        cc,...
        hand_type);
    
    F{i}.symbolic.lb = sym(F{i}.lb); % create symbolic lower bound
    F{i}.symbolic.ub = sym(F{i}.ub); % create symbolic upper bound
end

hand = mySGmakeHand(F,T);
hand.type = hand_type;
hand.phalanx_radius = FinTipDim/2; % link radius

hand = makePalm(hand); % palm of hand, struct

end
