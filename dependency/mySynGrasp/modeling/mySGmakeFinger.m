function finger = mySGmakeFinger(DHpars, base, q, idx,...
    lb, ub,...
    eval_type,...
    active_joints,...
    contacted_link,...
    cc,... % cylindrical coordinates
    hand_type)

    if nargin < 9 || isempty(contacted_link)
        contacted_link = zeros(size(DHpars,1)+1,1); % (n+1,1), index of contacted link. if link is in contact, then idx=1, otherwise 0. # link is one more than # DH params, with end point (fingertip) being considered as one extra link, and link with length=0 is consiered as virtual link
    end
    if nargin < 8 || isempty(active_joints)
        active_joints = ones(size(q)); % (n,1), flag to indicate if the joint is active (1) or not (0)
    end
    if nargin < 7 || isempty(eval_type)
        eval_type = 'num'; % calculate using numerical values of joint angles by default
    end
    
    if nargin < 6 || isempty(ub)
        ub = pi/2*ones(size(q));
    end
    
    if nargin < 5 || isempty(lb)
        lb = zeros(size(q));
    end

    if nargin < 4 || isempty(idx)
        idx = 0; % index of current finger in the hand. e.g. thumb: idx=1, little finger: idx=5
    end
    
    checkParams(DHpars, base, q);
    n = size(DHpars,1);
    
    q_sym = sym(['q',num2str(idx),'%d'],[1,n]); % create a list of sym variables for joint angles, [qi1,qi2,...], where i is the idx of fingers, '1' for thumb and '5' for little finger
    
    finger = struct('n', n,... % number of joints
        'idx', idx,... % index of current finger in the hand
        'joints_pos', [],... % (3,n+1) position of joints, will be updated in 'mySGmakeHand -> hand = mySGjoints(hand)'
        'tip', [],... % 4 x 4 HT from finger base to fingertip, calculated in 'moveFinger'
        'nlink', size(DHpars,1)+1,... % number of (virtual) links, each joint is the base of one link, FIRST LINK IS BASE (EMPTY), AND USE END_EFFECCTOR AS THE LAST LINK, so total number of link is size(DHpars,1)+1. Number of real link should be finger.n-1, or equivalent finger.nlink-2.
        'DHpars', DHpars,... % (n,4): n (number of joints) x 4 matrix containing Denavit Hartenberg
        'base', base,... % 4 x 4 homogeneous transformation matrix for the finger frame
        'q', q,... % (n,1) vector containing values of joint variables
        'q_sym', q_sym,... % symbolic form of joint angles
        'lb', lb,... % (n,1) default lower bounds of joint angles (will be updated in the construction of hand model (e.g. mySGmakeHand))
        'ub', ub,... % (n,1) default upper bounds of joint angles (will be updated in the construction of hand model (e.g. mySGmakeHand))
        'qin', (1:length(q))',... % (n,1) vector, indexing of joint 'q' % [deprecated] finger.qin = cumsum(ones(size(q))); % index of joint in the finger (1:max. number of q)
        'active_joints', active_joints,... % (n,1), index of active joints    
        'contacted_link', contacted_link,... % (n,1), index of contacted link
        'symbolic', [],... % struct, symbolic form of expressions (will be updated in moveFinger)
        'cp', [],... % (7,Nc), save contact points on this finger [old]
        'cc', cc,... % struct, cylindrical coordinates
        'hand_type', hand_type); % hand of finger
    
    % Generate LINK Expression for Finger
    if ~isfield(finger,'Link')
        finger = addLinkToFinger(finger);
    end
    
    % Generate Symbolic Expression for Finger
    if isempty(finger.symbolic)
        finger = calcSymbolicLinkTransformation(finger);
    end

    finger = moveFinger(finger, q, eval_type); % This is the initial moving of fingers
end