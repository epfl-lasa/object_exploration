%%% This function determines which fingers have reachability maps that are
%%% in contact with the object. Such regions indicate reachable places for
%%% corresponding finger links.

% object: described by point cloud (struct)
% map_hand: cell array, as calculated in reachability map
% links2plot: cell, describe the fingers and links that to be analized

function detect_contact_region(object, map_hand, links2plot)

    nVF = numel(links2plot); % number of virtual fingers (including palm)
    nF = nVF - 1; % number of fingers
    
    total_link = 0; % number of links in total
    for f = 1:nVF
        total_link = total_link + length(map_hand{f}); % Should be 21 (incl. palm)
    end

    ox = object.Location(:,1); % (N,1), coordinates of object point cloud
    oy = object.Location(:,2); 
    oz = object.Location(:,3);
    
    nSamples = size(object.Location,1);

    cmap = distinguishable_colors(total_link+1); % (21+1,3), color map, last color used to plot the non-contacted points

    inShapeSet = zeros(nSamples,1); % Index set of point cloud data inside rmap regions (0 or 1)
    
    distanceSet = inf(nSamples,1); % register the distance of this point to the geometrical center of corresponding rmap
    linkIndexSet = zeros(nSamples,1); % a list of indices to indicate to which link's rmap does this point belong to
    
    figure, hold on;
    
    for f = 1:nVF
        finger_links = links2plot{f}; % indices for finger links
        if isempty(finger_links) % do not plot this finger
            continue;
        end
        
        map_finger = map_hand{f};
        if isa(map_finger,'struct') % palm index: f = 5 (last)
            continue; % Skip palm for now
        end

        nlink = length(map_finger); % number of links in the finger to analyze
        for l = 1:nlink-1
            if ismember(l,finger_links) || any(isnan(finger_links))
                link_idx = (f-1)*4 + l;
                
                disp(link_idx);
                
                map_link = map_finger{l};
                map_dataset = map_link.linkmesh;

                px = map_dataset(:,1); % optional: down-sampling to plot fewer points
                py = map_dataset(:,2);
                pz = map_dataset(:,3);
                
                geoCenter = mean(map_dataset, 1); % the geometric center of this dataset [todo] Change to link-point distance
                shp = alphaShape(px,py,pz,20); % Set last parameter as Inf to produces the convex hull
                inShape_bool = inShape(shp,ox,oy,oz); % (N,1)

                if any(inShape_bool)
                    overlapIdx = inShapeSet & inShape_bool; % In case the current inShape points are already in another set (or more sets)
                    inShape_index = find(inShape_bool);
                    for i = 1:nume(inShape_index)
                        idx = inShape_index(i);
                        point = object.Location(idx,:); % (1,3)
                        d = norm(point(:) - geoCenter(:)); % distance to geometric center
                        if (overlapIdx(idx) && d <= distanceSet(idx)) || (~overlapIdx(idx))
                            distanceSet(idx) = d;
                            linkIndexSet(idx) = link_idx;
                        end
                    end
                    inShapeSet = inShapeSet | inShape_bool; % register the point cloud points inside in-rmap
                end
                
                % fprintf('Finger %d Link %d contacts the object.\n', f, l);
                plot(shp,'Facecolor',cmap(link_idx,:),'EdgeAlpha',0.10,'FaceAlpha',0.25);
                hold on;
            end
        end
    end
    
    assert(sum(linkIndexSet & ~inShapeSet) == 0);
    assert(sum(linkIndexSet | ~inShapeSet) == nSamples);
    
    % Visualization
    contactedLinks = unique(linkIndexSet);
    contactedLinks(~contactedLinks) = []; % remove '0' from link index set
    for i = 1:numel(contactedLinks)
        link_idx = contactedLinks(i);
        idx = linkIndexSet == link_idx;
        scatter3(ox(idx),oy(idx),oz(idx),'MarkerEdgeColor','none','MarkerFaceColor',cmap(link_idx,:));
        hold on;
    end
    
    scatter3(ox(~inShapeSet),oy(~inShapeSet),oz(~inShapeSet),'MarkerEdgeColor','none','MarkerFaceColor',cmap(end,:)); % plot remainning surface region (point cloud data that are not inside any rmaps)

    grid on;
    axis on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');

    title('Contact regions on object surface');
    hold off;

    disp('Finished');
    
end