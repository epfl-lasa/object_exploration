function fig = plotAvlContacts(P, V, fig)
% Plot Available contact positions and associated reference frames (or surface normals)
% Input:
%     * fig: handle of current plotting figure
%     * P: (contact) positions, (3*N), N points
%     * V: normal vectors to plot, denotes as tip points, (3*N), N vectors associated with N points in P
%       V should have the same size as P.
% Output:
%     * fig: handle of plotted figure

if nargin == 3 % handle exists
    figure(fig);
end

hold on;

N = size(P,2); % number of points to plot

sz = 100; % marker size
scale = 5; % radius as scaling ratio

% Scatter the (contact) point
scatter3(P(1,:),P(2,:),P(3,:),sz,[0.4660, 0.6740, 0.1880],'filled');

if nargin == 2 % Required to plot vectors (V is given)
    if iscell(V) % Multiple vectors for one single point
        
        x = P(1,:);
        y = P(2,:);
        z = P(3,:);
        
        % X vectors
        vx = scale*V{1};
        u = vx(1,:);
        v = vx(2,:);
        w = vx(3,:);
        quiver3(x,y,z,u,v,w,'Color','r','LineWidth',2.0);
        
        % Y vectors
        vy = scale*V{2};
        u = vy(1,:);
        v = vy(2,:);
        w = vy(3,:);
        quiver3(x,y,z,u,v,w,'Color','g','LineWidth',2.0);
        
        % Z vectors
        vz = scale*V{3};
        u = vz(1,:);
        v = vz(2,:);
        w = vz(3,:);
        quiver3(x,y,z,u,v,w,'Color','b','LineWidth',2.0);

    else % one normal vector for one point 
        x = P(1,:);
        y = P(2,:);
        z = P(3,:);
        u = scale*V(1,:);
        v = scale*V(2,:);
        w = scale*V(3,:);
        quiver3(x,y,z,u,v,w,'LineWidth',2.0,'Color',[0.9290, 0.6940, 0.1250]);
    end
end

if nargin < 3
    fig = true; % just return a flag
end

end