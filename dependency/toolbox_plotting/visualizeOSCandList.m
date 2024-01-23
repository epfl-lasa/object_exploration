function visualizeOSCandList(hand, osCandList, identifier, if_save)

if nargin < 4
    if_save = false;
end
if nargin < 2
    osCandList = {};
end
if nargin < 1
    models = load('models.mat');
    hand = models.hand;
end

if ~hand.reachability_map
    error('Incomplete hand model.');
end

cmap = distinguishable_colors(10);
cidx = 0;

for j = 1:numel(osCandList)
    this_os = osCandList{j};
    os_info = this_os.os_info;
    os_rmap = this_os.os_rmap;
    % os_dist = this_os.os_dist;
    
    figure, hold on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    view([-150 30]);
    
    axis equal;
    grid on;
    SGplotHand(hand); hold on;
    
    fig_title = [identifier,'_os_candidate_',num2str(j)];
    
    for i = 1:numel(os_info)
        temp_info = os_info{i};
        
        [idx_f,idx_l] = deal(temp_info(1),temp_info(2));
        fig_title(end+1:end+5) = ['F',num2str(idx_f),'L',num2str(idx_l),'_'];
        
        %%% Alternative
%         temp_rmap = hand.F{idx_f}.Link{idx_l}.map_link.linkmesh;
        
        temp_rmap = os_rmap{i};  % (Nsample,3)
        px = temp_rmap(:,1); % optional: down-sampling to plot less
        py = temp_rmap(:,2);
        pz = temp_rmap(:,3);
        
        cidx = cidx + 1;
        
        if all(temp_info) % normal finger links
            shp = alphaShape(px,py,pz,Inf); % Inf: produces the convex hull
            plot(shp,'Facecolor',cmap(cidx,:),'EdgeColor','k','EdgeAlpha',0.5,'FaceAlpha',0.5);
        else % if 0 exists in temp_info, palm
            scatter3(px,py,pz,'MarkerFaceColor','k');
        end
        hold on;
    end
    
    if if_save
        savefig([fig_title,'.fig']);
    end
    hold off;
end