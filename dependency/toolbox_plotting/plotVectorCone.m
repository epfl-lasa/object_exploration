function plotVectorCone(cp,S,c,kappa)
% plot a group of vectors, given the contact point cp, vectors S(3,k), and
% scaling factor kappa
    if nargin < 4
        kappa = 10;
    end
    if nargin < 3
        c = 'r';
    end
    
    axis equal;
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
    scatter3(cp(1),cp(2),cp(3),100,'g','filled'); % contact point
    k = size(S,2);
    C = repmat(cp(:),1,k);
    S = S.*kappa; % S are only directions of vectors
    E = C + S; % End points of cone
    X = [C(1,:); E(1,:)];
    Y = [C(2,:); E(2,:)];
    Z = [C(3,:); E(3,:)];
    plot3(X,Y,Z,'Color',c,'LineWidth', 2.0);
end