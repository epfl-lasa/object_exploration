function hand = makePalm(hand)
% Construct the model of hand palm. Currently support:
% 20dof human hand (mySGparadigmatic)
% Modified from 'mySGplotPalm'.
% output:
%   X: points of constructing the palm
%   k: the convex hull constructed by the points X

% param = load('problem_config.mat','Sp','k','f_gamma');
global fc_k fc_Sp f_gamma

if isempty(fc_k) || isempty(fc_Sp) || isempty(f_gamma)
    configuration;
end 
    
k = fc_k;
Sp = fc_Sp;

% k = param.k;
% f_gamma = param.f_gamma;
% Sp = sym(param.Sp);

x0 = [sym('x');sym('y');sym('z')]; % sym of object center

nps = 5; % nps is the resolution of sphere (joint model)

radius = hand.phalanx_radius; % radius of sphere (joint model)

T = hand.T;
F = hand.F;
% n = hand.n; % number of fingers
% hand_type = hand.type;

% The origin of the palm base frame is calculated as the mean of all finger
% base points
palm_base = T*[0 -1 0 0;... % World to palm base
    1 0 0 0;
    0 0 1 0;
    0 0 0 1];

%%% NOTICE: FINGERS MUST BE ORDERED IN SEQUENCE: T,I,M,R,L
% thumb_base = F{1}.base(1:3,4); % base of thumb
TL = F{2}.base(1:3,4); % top left corner, base of index
TR = F{end}.base(1:3,4); % top right, base of little finger
BL = TL;
BL(2) = F{1}.base(2,4); % bottom left
palm_ctr = (TR+BL)/2; % palm center. notice that bases of fingers have been already transformed
BR = 2*palm_ctr - TL; % bottom right

palm_base(1:3,4) = palm_ctr; % use the middle point of thumb and little finger

P.palm_base = palm_base;
P.palm_ctr = palm_ctr;

basepoints = [TL,TR,BR,BL]; % (3,N)
basepoints_h = cat(1,basepoints,ones(1,size(basepoints,2))); % homogeneous coordinates, (4,n+1)

bp = basepoints';
co = mean(bp);
for i = 1:size(bp,1)
    ncp(i,:) = (co - bp(i,:))/norm(co - bp(i,:));
end
X = bp;
for i = 1:size(bp,1)
    [filletx,fillety,filletz] = sphere(nps);
    spherecenter = bp(i,:) + radius*ncp(i,:);
    for j = 1:nps+1
        for m = 1:nps+1 % k is used as the edges of approximated friction cone
            fillett = radius*[filletx(j,m) fillety(j,m) filletz(j,m)]+spherecenter;
            X = [X;fillett];
        end
    end
end

%%% 3D palm
X = unique(X,'rows');
[K,V] = convhulln(X); % The convex hull of palm
P.X = X;
P.K = K;
P.V = V; % volume

%%% From palm center to inner surface center
HTctr2inr = trvec2tform([0,0,-1]*radius); % reference frame on the front surface of the palm. pointing to the Z- direction, translation for d = radius

%%% [Explanation] point transformation
%{
pa = H_a_b * pb
thus, to transform finger base points from original palm model (T = eye(4), world CF) to surface of palm (palm surface CF),
first calculate the H matrix:
H = eye(4)
First step, from world transfer to hand CF (left product: w.r.t. world CF)
H = T*eye(4)
Second, from hand CF to front surface of palm (right prod.: w.r.t. current CF)
H = T*eye(4)*HT2inr

Thus, the following equation holds:
points_inr = H * {F{i}.base, i = 1,...,nf}

Since: basepoints_h{:,i} = T * F{i}.base;
Thus: {F{i}.base, i = 1,...,nf} = inv(T)*basepoints_h{:,i}

To summary:
points_inr = T * eye(4) * HT2inr * inv(T) * basepoints_h{:,i}
points_inr = T * HT2inr * inv(T) * basepoints_h{:,i}
%}

points_inr = T * HTctr2inr * inv(T) * basepoints_h; % transfer w.r.t. current coordinate frame
points_inr = points_inr(1:3,:); % transformed points, constructing the inner side of the palm
points_inr = transpose(points_inr); % (N,3) same as the data dim of convhull

HTbase2inr = palm_base * HTctr2inr; % from base to intra face center RF
% ctr_inr = HTbase2inr(1:3,4); % palm center of inner surface
R = HTctr2inr(1:3,1:3);
t = HTctr2inr(1:3,4); % translation vector, from palm base to palm inner surface
ctr_inr = t + R*palm_cntr(:);

S_inr = calcPolygonArea(points_inr,ctr_inr); % Notice that points in points_inr must be ordered in clockwise or anti-clockwise order

P.S_inr = S_inr; % S_inr: inner surface area, is used to check the 'inside-convex hull' constraints later
P.points_inr = points_inr; % vertices of inner area
P.HTbase2inr = HTbase2inr;
P.ctr_inr = ctr_inr; % palm inner surface center

%%% Construct contact point symbolic fields
% p = [sym('px');sym('py');sym('pz')]; % contact point coordinate
% P.contact.symbolic.p = p;
% P.contact.symbolic.p_area = calcPolygonArea(points_inr,p); % used to check the 'in-polygon' condition abs(p_area-P.S_inr)<1e-4
% P.contact.symbolic.n = sym(-palm_base(1:3,3)); % pointing towards Z- direction (inner side of palm)

%%%%%%%%%%% Approximation of Friction Cone %%%%%%%%%%
% HTinr2cp = sym(eye(4)); % transform from palm front center to contact point, only a translation
% HTinr2cp(1:3,4) = p(:) - palm_ctr_inr(:); % from inner surface RF to contact point RF

pvec = [sym('vx'),sym('vy')]; % (vx,vy) defined in WORLD reference frame. this vector transforms from palm ctr inr to contact point on inr surface
HTinr2cp = sym(eye(4)); % local transform, move on X-Y plane of HTbase2inr
HTinr2cp(1:2,4) = pvec(:);

%%% Since the translation vector is defined in WCF, the prod. should be on the left side
HTcp = HTbase2inr * HTinr2cp; % change from world to local contact reference frame
p = HTcp(1:3,4);
P.contact.symbolic.p = p;
P.contact.symbolic.p_area = calcPolygonArea(points_inr,p);
P.contact.symbolic.n = -HTcp(1:3,3);

P.contact.symbolic.vectors = points_inr - p(:).'; % (4,3) - (1,3), vectors pointing from p to vertices, used to determine if the vector is inside the polygon by solving linear equation

%%% obtain lower and upper bound of pvec
norm_vtx = bsxfun(@minus, points_inr, ctr_inr(:).'); % (N,3), normalized vertices represented in world CF
% P.pvec_ub = max(norm_vtx,[],1); % [vx,vy]
% P.pvec_lb = min(norm_vtx,[],1);

%%% calculate the upper bound and lower bound of pvec. noticed that pvec is
%%% defined in the palm inner surface reference frame insead of world
%%% reference frame, so vx along the short axis, vy the longer one

norm_vtx_palm = transpose(palm_base(1:3,1:3) * norm_vtx.'); % represent the bounds limits from world coordinate frame to local palm CF to match the definition of p = [vx,vy]
P.pvec_ub = max(norm_vtx_palm,[],1);
P.pvec_lb = min(norm_vtx_palm,[],1);

R = HTcp(1:3,1:3);
FC = R * Sp; % Should use the friction cone of palm contact, contact normal: in Z- direction of local CF
P.contact.symbolic.FC = FC;
% plotVectorCone(ctr_inr,FC,'r',10); % plot vector cone

d = p(:)-x0(:);
r_mtx = sym(repmat(d,1,k));
TC = cross(r_mtx,FC);
P.contact.symbolic.TC = TC;

vn_i = -d; % (3,1) % from contact point, pointing towards object center
vn_i = vn_i./sqrt(ones(1,3)*(vn_i.*vn_i)); % vectorized form of normalization, equivalent to: vn_i = vn_i./norm(vn_i);
TC_torsinal = f_gamma*vn_i; % Torsinal torque (soft finger)
P.contact.symbolic.TC_torsinal = TC_torsinal;

% points used for meshgrid sampling
P.basepoints_h = basepoints_h;
P.HT2inr = HTctr2inr; % plotCoordinateFrame(HT,'r',10)
P.contact.symbolic.HTcp = HTcp; % from world CF to contact CF

P.thickness_half = radius;

hand.P = P;
hand.pactv = true; % if palm is active (available for grasping)

% Obtain sample points on palm for plotting
hand = obtainPalmCvxHullVertices(hand);

end