%    mySGallegroLeft - The function builds the Allegro Hand (SimLab) model (left)
%
%    Usage: hand = mySGallegroLeft
%
%    Returns:
%    hand = the AllegroHandLeft hand model
%
%    Contributors:
%    M. Malvezzi, V. Ruiz Garate, M. Pozzi
%
%    References:
%    1. V. Ruiz Garate, M. Pozzi, D. Prattichizzo, A. Ajoudani. A Bio-Inspired 
%    Grasp Stiffness Control for Robotic Hands. Frontiers in Robotics 
%    and AI,2018. 
%    2. V. Ruiz Garate, M. Pozzi, D. Prattichizzo, N. Tsagarakis, A.  
%    Ajoudani. Grasp Stiffness Control in Robotic Hands through Coordinated  
%    Optimization of Pose and Joint Stiffness. IEEE Robotics and Automation 
%    Letters, 3(4):3952-3959, October 2018.

function hand = mySGallegroLeft(T)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    1 - A L L E G R O  H A N D  D H  P A R A M E T E R S
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if nargin < 1
    T = eye(4);
end

hand_type = 'AllegroHandLeft';

FinTipDim = 28; % diameter of finger

DHpars{4} = [];
base{4} = [];
F{4} = [];

%%% Attention: Original SynGrasp code defines finger ordering as: 1-index,
%%% 2-middle, 3-ring, 4-thumb. Here, I changed thumb as the 1st finger to
%%% keep consistency with other hand model. Index, middle, and little
%%% finger corresponds to the 2nd, 3rd, and 4th finger, respectively.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1: Thumb
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x11 = 16.958;
y11 = -73.288;
z11 = 18.2;

x12 = 72.147;
y12 = -78.116;
z12 = 13.2;

x13 = 72.147;
y13 = -78.116;
z13 = 13.2;

x14 = 123.351;
y14 = -82.596;
z14 = 13.2;

base{1} = [1 0 0 x11;
    0 1 0 y11;
    0 0 1 z11;
    0 0 0 1];

rotthumb = -5*pi/180;
base{1}(1:3,1:3) = SGrotz(rotthumb)*SGrotz(-pi/2)*SGroty(-pi/2);

d11 = -sqrt((z11-z12)^2+(y11-y12)^2);

a11 = z11-z12;

x12dh = x11 -d11*sin(rotthumb);
y12dh = y11 +d11*cos(rotthumb);

a12 = -sqrt((x12-x11)^2+(z12-z11)^2);
a13 = sqrt((x14-x13)^2+(y14-y13)^2);

a14 = 59.3;

% [K. Yao] Comments: definition of DHpars mtx:
% * DHpars(i,:): DH-params of the ith link
% * DHpars(:,3): rotation angles
% * order of parameters in each row:
%   * alpha: the angle between z_i-1 and z_i rotated around x_i
%   * a: distance along x_i from the intersection to o_i
%   * theta: the angle between x_i-1 and x_i rotated around z_i-1 (axis of motion)
%   * d: distance along z_i-1 from o_i-1 to the intersection of the x_i and z_i-1 axes

DHpars{1} = [pi/2 a11 0 d11; % alpha, a, theta, d
    pi/2 0 pi/2 a12;
    0 a13 -pi/2 0;
    0 a14 0 0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2: Index Finger
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x21 = 45.098;
y21 = 14.293;

a22 = 54;
a23 = 38.4;
a24 = 43.7;

rotbaseindex = -5*pi/180;
Rindex = SGrotz(rotbaseindex)*SGrotz(-pi/2)*SGroty(-pi/2);

base{2} = [1 0 0 x21;
    0 1 0 y21;
    0 0 1 0;
    0 0 0 1];
base{2}(1:3,1:3) = Rindex;

DHpars{2} = [pi/2 0 0 0;
    0 a22 pi/2 0;
    0 a23 0 0;
    0 a24 0 0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3: Middle Finger
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x31 = 0;
y31 = 16.6;

a32 = a22;
a33 = a23;
a34 = a24;

Rmiddle = SGrotz(-pi/2)*SGroty(-pi/2);

base{3} = [1 0 0 x31;
    0 1 0 y31;
    0 0 1 0;
    0 0 0 1];
base{3}(1:3,1:3) = Rmiddle;

DHpars{3} = [
    pi/2 0 0 0;
    0 a32 pi/2 0;
    0 a33 0 0;
    0 a34 0 0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 4: Little Finger
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x41 = -45.098;
y41 = 14.293;

a42 = a22;
a43 = a23;
a44 = a24;

rotbaselast = 5*pi/180;
Rlast = SGrotz(rotbaselast)*SGrotz(-pi/2)*SGroty(-pi/2);

base{4} = [1 0 0 x41;
    0 1 0 y41;
    0 0 1 0;
    0 0 0 1];
base{4}(1:3,1:3) = Rlast;

DHpars{4} = [
    pi/2 0 0 0;
    0 a42 pi/2 0;
    0 a43 0 0;
    0 a44 0 0];

%%%%%%%%%%%%%%%%%%%%
% Make Hand
%%%%%%%%%%%%%%%%%%%%
cc = struct('rho',FinTipDim/2,'phi',0,'alp',0.5); % default parameters that initialize the cylindrical coordinates describing contact point on link

for idx = 1:length(DHpars)
    % lower bound and upper bound values obtained from official website
    if idx == 1 % thumb
        lb = [0.3635738998060688, -0.20504289759570773, -0.28972295140796106, -0.26220637207693537];
        ub = [1.4968131524486665, 1.2630997544532125, 1.7440185506322363, 1.8199110516903878];
    else
        lb = [-0.59471316618668479, -0.29691276729768068, -0.27401187224153672, -0.32753605719833834];
        ub = [0.57181227113054078, 1.7367399715833842, 1.8098808147084331, 1.71854352396125431];
    end
    
    q = (lb(:)+ub(:))/2;
    
    F{idx} = mySGmakeFinger(DHpars{idx},T*base{idx},q,idx,...
        lb,ub,...% lb, ub
        '',[],[],...% eval_type, active_joints, contacted_link
        cc,...
        hand_type); % hand type the finger belongs to. Needs in the function 'moveFinger'.
end

hand = mySGmakeHand(F, T);

hand.type = hand_type;
hand.phalanx_radius = FinTipDim/2; % radius of hand link
hand = makePalm(hand); % palm of hand, struct

end