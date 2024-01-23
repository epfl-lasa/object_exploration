%%% Search for collision rmaps for each finger link

function [hand, CollMap] = collisionOSSearch(hand, rmap, if_plot, if_save)
if nargin < 4
    if_save = true;
end
if nargin < 3
    if_plot = true;
end
if nargin < 2
    rmap = load('reachable_map.mat');
    rmap = rmap.map_hand;
    %{
    rmap is organzied as a cell for each finger:
    [{str(empty), str, str, str, str(tip)},... % rmap of finger 1: link 1, link 2,...
    {...},... % finger 2
    {...},... % finger 3
    {...},... % finger 4
    {...},... % finger 5
    {str}] % palm
    %}
end
if nargin < 1
    models = load('models.mat');
    hand = models.hand;
end

% param = load('problem_config.mat', 'hand_radius');
hand_radius = hand.hand_radius;
fprintf('* Construct collision map for hand radius: %d\n', hand_radius);

d = hand_radius * 2; % collision distance is twice the link cylinder radius
nf = numel(rmap); % 6, number of fingers (incl. palm)

link_dict = {}; % save each one of the (1) link rmap, (2) link info, (3) link name, in a list
% e.g. thumb-1st link (this one is in fact virtual), is [1,1], middle-2nd link is [3,2]
category_list = {}; % save the string of category name for plotting

for f = 1:nf % iterate over fingers
    temp_rmap = rmap{f}; % rmap of the current finger
    
    if isa(temp_rmap,'struct') % this is palm, corresponds to f==6
        link_dict{end+1} = struct('rmap', temp_rmap.linkmesh,... % (N,3)
            'info', [0,0]);
        category_list{end+1} = 'P';
        
    elseif isa(temp_rmap,'cell') % rmap of finger, contains rmaps of links
        nl = length(temp_rmap)-1; % number of links (incl. virtual links), ignore the last one (fingertip, not real link)
        for l = 1:nl
            l_rmap = temp_rmap{l}; % j_map is a struct
            if size(l_rmap.linkmesh,1) == 1 % when l==1, the virtual link, skip virtual link (link length = 0, results in a singular rmap, e.g. the 1st link in the model is virtual)
                continue;
            else
                link_dict{end+1} = struct('rmap', l_rmap.linkmesh,... % l_rmap, struct, has fields: p, r, n, linkmesh, cnvxIndices, bndryIndices
                    'info', [f,l]);
                category_list{end+1} = [num2str(f),'-',num2str(l)];
            end
        end
    end
end

N = numel(link_dict); % total number of link (+palm) rmaps, should be 5*3+1 = 16
CollMap = cell(1,N); % to save the collision info for each link
existence_heatmap = zeros(N); % heatmap to save the number of collisions for each link

for i = 1:N % this link, link_i
    rmap_i = link_dict{i}.rmap; % (N_samples,3)
    info_i = link_dict{i}.info; % (1,2)
    
    if ~all(info_i)
        disp(' palm');
    else
        fprintf(' F%d L%d\n', info_i(1), info_i(2));
    end
    
    link_j_list = {}; % information of all link_j that collide with link_i
    for j = i+1:N % j: index of link that may collide with link i
        rmap_j = link_dict{j}.rmap;
        info_j = link_dict{j}.info;
        
        % Extract data points on the boundary surface of the rmaps to
        % accelerate computation
        k_i = boundary(rmap_i); rmap_i = rmap_i(k_i,:);
        k_j = boundary(rmap_j); rmap_j = rmap_j(k_j,:);
        
        % Notice that consequtive links cannot be skipped. Could overlap in
        % grasping planning.
        dist = pdist2(rmap_i, rmap_j, 'euclidean');
        min_dist = min(dist(:));
        
        if min_dist < d % minimum distance between rmaps is larger than 2*link radius, potential collision exists            
            existence_heatmap(i,j) = d - min_dist; % the lighter the color, the more likely the collision
            link_j_list{end+1} = info_j; % (1,2) add jth link to the collision list of i
        end
        clear('rmap_j');
    end
    clear('rmap_i');
    
    if all(info_i)
        hand.F{info_i(1)}.Link{info_i(2)}.collList = link_j_list; % add to the 
    else % palm
        hand.P.collList = link_j_list;
    end
    
    if nargout > 1
        CollMap{i}.link_info = info_i; % (1,2)
        CollMap{i}.coll_list = link_j_list; % a list containing information of all links that collide link i
    end
end

hand.collision_map = 'true'; % set flag

%%% plot overview of existence
if if_plot
    figure;
    heatmap(existence_heatmap,...
        'XData',category_list,...
        'YData',category_list);
    title(['Cartesian-space collision between links (r: ', num2str(hand_radius), ')']);
    xlabel('Finger-Link pair');
    ylabel('Finger-Link pair');
    set(gca, 'FontSize', 14);

    saveas(gca, ['../database/results/collision_heatmap_',num2str(hand_radius),'.jpg'], 'jpeg');
    saveas(gca, ['../database/results/collision_heatmap_',num2str(hand_radius),'.eps'], 'epsc');
end

%%% save results
if if_save
    save(['../database/collision_map_',num2str(hand_radius),'.mat'],'CollMap');
    fprintf('File saved: collision_map_%d.mat\n', hand_radius);
end

end