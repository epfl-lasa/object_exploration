function plotCoordinateFrame(H,c,kappa)

    % kappa: scaling factor
    if nargin < 3
        kappa = 10;
    end
    if nargin < 2
        c = 'k';
    end

    axis equal;
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');

    ori = H(1:3,4);
    R = H(1:3,1:3);
    scatter3(ori(1),ori(2),ori(3),100,'g','filled');
    C = repmat(ori(:),1,3); % center
    E = C + R.*kappa; % end of coordinate vector axes

    X = [C(1,:); E(1,:)];
    Y = [C(2,:); E(2,:)];
    Z = [C(3,:); E(3,:)];

    plot3(X,Y,Z,'Color',c,'LineWidth', 2.0);

end