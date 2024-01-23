% This function searches for potential opposition spaces that may be used
% to fit in the given object.
% Criterion: exist a pair of at least two points in two rmap (of 2 links,
% respectively), that in the opposite direction
% 
% Input:
%     * rmap: {1,nf}, reachability map of each joint and link of the entire hand
%     * object: the object want to fit in
% Output:
%     * OS.os_rmap: {1,nos}, length is the number of opposition space. Each component of os_rmap contains all rmap groups of one opposition spaces.
%     * OS.os_info: {1,nos}, contains information of the link os_rmap within each opposition space

function [OS, existence_heatmap] = fitInOppsitionSpace(rmap, object, if_plot_trial, if_save)
if nargin < 4
    if_save = true;
end
if nargin < 3
    if_plot_trial = false; % plot result of each feasible trial
end

%% Step 1: Unpack data, make a list of all joint rmaps and construct link rmap
nf = numel(rmap); % number of fingers, each finger has one rmap
link_dict = {}; % save each one of the link rmap in a list, and also the corresponding information of each link rmap. e.g. thumb-1st link (this one is in fact virtual), is [1,1], middle-2nd link is [3,2]
category_list = {}; % save the string of category name

% fingerName = {'T','I','M','R','L'};
% phalangeName = {'M','P','I','D'}; % stands for: metacarpals, proximal phalanges, intermediate phalanges, distal phalanges
for f = 1:nf % iterate over fingers
    f_rmap = rmap{f}; % rmap of the current finger
    if isa(f_rmap,'struct') % palm
        link_dict{end+1} = struct('mesh',f_rmap.linkmesh,...
            'info',[0,0]);
        category_list{end+1} = 'P';
    else
        nl = length(f_rmap)-1; % number of links (incl. virtual links), ignore the last one (fingertip, not real link)
        for l = 1:nl
            l_rmap = f_rmap{l}; % l_map is a struct
            if size(l_rmap.linkmesh,1) == 1 % skip virtual link (link length = 0, results in a singular rmap, e.g. the 1st link in the model is virtual)
                continue;
            else
                link_dict{end+1} = struct('mesh',l_rmap.linkmesh,... % l_rmap, struct, has fields: p, r, n, linkmesh, cnvxIndices, bndryIndices
                    'info', [f,l]);
                % category_list{end+1} = [fingerName{f},'-',phalangeName{l}]; % in the form of f-l, for example, thumb-2nd link is '1-2'; middle-3rd is '3-3'
                category_list{end+1} = [num2str(f),'-',num2str(l)];
            end
        end
    end
end

%% Step 2: find pairs of joint rmaps that fit the object model
OS = {}; % (1,nos), each cell contains information of one feasible opposition spaces, and save the corresponding opposition spaces, composing of two link rmaps

nm = numel(link_dict); % number of meshgrid (#link*#fingers+palm)
cmap = distinguishable_colors(nm);

fprintf('[fitInOppsitionSpace] Object radius: %d\n', object.radius);

% d = 2*object.radius; % diameter of the object
const = load('problem_config.mat','f_mu'); % load constant values from config files
d = 2 * object.radius * cos(atan(const.f_mu)); % consider the influence of friction cone

existence_heatmap = zeros(nm); % construct a heatmap to indicate the existance of opposition space between finger links

for i = 1:nm
    % Iterate over all rmaps, rmap_i
    rmap_i = link_dict{i}.mesh; % (N,3)
    info_i = link_dict{i}.info; % (1,2)
    
    % Notice that Thumb 2nd link belongs to palm
    if size(rmap_i,1) == 1 % Skip virtual (singular) link (link length is 0)
        continue;
    end

    for j = i+1:nm
        % iterate over all rmaps, rmap_j, to check if it forms an
        % opposition space with rmap_i
        rmap_j = link_dict{j}.mesh;
        info_j = link_dict{j}.info; % (1,2)
        
        % Extract data points on the boundary sruface of the rmaps
        k_i = boundary(rmap_i);
        if numel(k_i) % k_i~=0, the convhull is not degenerated (palm is degenerated)
            rmap_i = rmap_i(k_i,:);
        end
        
        k_j = boundary(rmap_j);
        if numel(k_j) % k_j~=0, the convhull is not degenerated (palm is degenerated)
            rmap_j = rmap_j(k_j,:);
        end
        
        dist = pdist2(rmap_i, rmap_j, 'euclidean'); % taken elements as (1,N), point-wise distance
        max_dist = max(dist(:));
        min_dist = min(dist(:));
        
        if (min_dist<=d) && (max_dist>=d) % Criterion for deciding if two rmaps can intersect the object (min_dist<d), and if they have enough space to fit in the object when they widely open (max_dist>d)
            % fprintf('Exist feasible opposition space: F%dL%d and F%dL%d.\n',info_i(1),info_i(2),info_j(1),info_j(2)); % F0L0 is palm
            if_exist = true;
            
            % existence_heatmap(i,j) = max_dist - min_dist; % use the maximum distance difference
            existence_heatmap(i,j) = max_dist;
            
            data.os_info = {info_i, info_j};
            data.os_rmap = {rmap_i, rmap_j};
            data.os_dist = [min_dist, max_dist]; % min and max distance of this pair
            OS{end+1} = data;
        else
            if_exist = false;
            % fprintf('   No feasible opposition space: F%dL%d and F%dL%d.\n',info_i(1),info_i(2),info_j(1),info_j(2));
        end
        
        %% plotting all pairs of opposition spaces that can fit the object
        if if_exist && if_plot_trial
            figure, hold on;
            [k_i,~] = boundary(rmap_i,0.5); % Surf the boundaries volume. s=0.5 is the default shrink factor
            if numel(k_i)
                trisurf(k_i,rmap_i(:,1),rmap_i(:,2),rmap_i(:,3),'FaceColor',cmap(i,:),'FaceAlpha',0.5); hold on;
            else
                scatter3(rmap_i(:,1),rmap_i(:,2),rmap_i(:,3),'MarkerFaceColor',cmap(i,:)); hold on;
            end
            [k_j,~] = boundary(rmap_j,0.5);
            if numel(k_j)
                trisurf(k_j,rmap_j(:,1),rmap_j(:,2),rmap_j(:,3),'FaceColor',cmap(j,:),'FaceAlpha',0.5); hold on;
            else
                scatter3(rmap_j(:,1),rmap_j(:,2),rmap_j(:,3),'MarkerFaceColor',cmap(j,:));
            end
            
            % Scatter the pairs of points that result in min and max distances
            [min_i,min_j] = find(dist == min_dist);
            if length(min_i) > 1 || length(min_j) > 1 % if multiple pairs of points have minimal distance, only use the first one, as an example for illustration
                min_i = min_i(1);
                min_j = min_j(1);
            end
            p_min_i = rmap_i(min_i,:);
            p_min_j = rmap_j(min_j,:);
            scatter3([p_min_i(1),p_min_j(1)],[p_min_i(2),p_min_j(2)],[p_min_i(3),p_min_j(3)],50,'k','filled');
               plot3([p_min_i(1),p_min_j(1)],[p_min_i(2),p_min_j(2)],[p_min_i(3),p_min_j(3)],'Color','g','LineWidth',2.0);
            hold on;
            
            [max_i,max_j] = find(dist == max_dist);
            if length(max_i) > 1 || length(max_j) > 1 % if multiple pairs of points have minimal distance, only use the first one, as an example for illustration
                max_i = max_i(1);
                max_j = max_j(1);
            end
            p_max_i = rmap_i(max_i,:);
            p_max_j = rmap_j(max_j,:);
            scatter3([p_max_i(1),p_max_j(1)],[p_max_i(2),p_max_j(2)],[p_max_i(3),p_max_j(3)],50,'k','filled');
               plot3([p_max_i(1),p_max_j(1)],[p_max_i(2),p_max_j(2)],[p_max_i(3),p_max_j(3)],'Color','g','LineWidth',2.0);
            hold on;
            
            axis equal; grid on;
            xlabel('X'); ylabel('Y'); zlabel('Z');
            title(['F', num2str(info_i(1)), 'L', num2str(info_i(2)),' and F', num2str(info_j(1)), 'L', num2str(info_j(2))]);
            hold off;
        end
        clear('rmap_j');
    end
    clear('rmap_i');
end

%%% plot overview of existence
figure;
heatmap(existence_heatmap,...
    'XData',category_list,...
    'YData',category_list);
title(['Existence of Opposition Space (r: ', num2str(object.radius), ')']);
xlabel('Finger-Link pair');
ylabel('Finger-Link pair');
set(gca, 'FontSize', 14);

fprintf('[fitInOppsitionSpace] Total number of opposition spaces: %d\n', length(OS));
saveas(gca, ['../database/results/os_heatmap_',num2str(object.radius),'.jpg'], 'jpeg');
saveas(gca, ['../database/results/os_heatmap_',num2str(object.radius),'.eps'], 'epsc');
% close gcf;

if if_save
    save(['../database/opposition_space_',num2str(object.radius),'.mat'],'OS');
    fprintf('File saved: opposition_space_%d.mat\n',object.radius);
end
end