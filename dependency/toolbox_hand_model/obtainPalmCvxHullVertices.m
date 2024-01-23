% This function calculates the convexhull vertices of the palm, represented as a set of 3d points.
% Tested on Allegro Hand Left model. 

function [hand,cvx_vtx] = obtainPalmCvxHullVertices(hand)
    fprintf('Creating convex hull vertices of hand palm...');

    assert(isequal(hand.T,eye(4)));
    
    nps = 5;
    radius = hand.phalanx_radius;
    
    F = hand.F;
    n = hand.n;

    basepoints = zeros(3,n+1);

    for i = 1:length(F)
        basepoints(:,i) = F{i}.base(1:3,4);
        % The allegro hand palm is not a strict rectangular. Adjust a bit
        % here to approximate the real shape.
        % This adjustment works only for hand.T = eye(4). 
        if i == 2 % Index finger
            basepoints(1,i) = basepoints(1,i) + 0.95*radius;
            basepoints(2,i) = basepoints(2,i) - 0.20*radius;
        end
        if i == 4 % Ring finger
            basepoints(1,i) = basepoints(1,i) - 0.95*radius;
            basepoints(2,i) = basepoints(2,i) - 0.20*radius;
        end
    end

    c = length(F);
    if abs(hand.F{2}.base(1,4)-hand.F{c}.base(1,4)) > abs(hand.F{2}.base(2,4)-hand.F{c}.base(2,4))
        puntodx = [hand.F{c}.base(1,4)
            hand.F{1}.base(2,4)
            hand.F{c}.base(3,4)];
    else
        puntodx = [hand.F{1}.base(1,4)
            hand.F{c}.base(2,4)
            hand.F{c}.base(3,4)];
    end
    basepoints(:,n+1) = puntodx;

    % Add surface points used in constructed palm
    palm_samples = hand.T\hand.P.basepoints_h;
    basepoints = cat(2, basepoints, palm_samples(1:3,:)); % (3,N)

    % Create thickness of palm
    basepoints_front = basepoints;
    basepoints_front(3,:) = min(basepoints(3,:));
    basepoints_back = basepoints;
    basepoints_back(3,:) = max(basepoints(3,:));
    basepoints = cat(2,basepoints_front,basepoints_back); % (3,N+N)

    cp = basepoints'; % (18,3)
    co = mean(cp);

    ncp = (co-cp)./vecnorm(co-cp, 2, 2);

    X = cp;

    for i = 1:size(cp,1)
        [filletx,fillety,filletz] = sphere(nps);
        spherecenter = cp(i,:) + radius*ncp(i,:);
        for j = 1:nps+1
            for k = 1:nps+1
                fillett = radius*[filletx(j,k) fillety(j,k) filletz(j,k)] + spherecenter;
                X = [X;fillett];
            end
        end
    end

    % k = convhulln(X); % X: (666,3)
    cvx_vtx = transpose(X); % (3,N), palm convex hull's vertices, transpose for consistency
    cvx_vtx_h = cat(1,cvx_vtx,ones(1,size(cvx_vtx,2))); % (4,N), homogeneous points
    hand.P.cvx_vtx_h = cvx_vtx_h;
    fprintf('done.\n');
end