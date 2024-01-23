function plotReachabilityMap(hand, hand_map, links2plot)
% plot the reachability map
% map_hand: the reachability map
% finger_idx: cell, {}, 1*nf, indices, map_idx{i} indicates the indices of
% links of finger i. if NaN, plot all links.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if nargin < 3
    links2plot = repmat({NaN},1,numel(hand_map)); % NaN: plot all links of all fingers, number of fingers (palm is counted as one finger)
    % link_idx_cell = {[],[4],[],[],[]}; % real link for paradigmatic hand:
    % [2,3,4], last finger is palm. can use arbitrary number, e.g. [1].
    
    % link_idx_cell = {[1,2,3,4],[2,3,4],[3,4],[4],[1]}, link_idx_cell{i}
    % indicates the link indices on the ith finger to plot
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nf = numel(links2plot); % number of fingers (to plot)

total_link = 0; % number of links in total
for f = 1:nf
    total_link = total_link + length(hand_map{f});
end

if isempty(links2plot)
    return;
end

% cmap = cool(total_link); % generate different colors for each finger, using colormap 'hot'
cmap = distinguishable_colors(total_link);

%%% plot hand model
mySGplotHand(hand); % plot robotic hand at its initial position
hold on;

cidx = 1; % cidx = 4+4; % color index. counter of link to plot, used to select colors
for f = 1:nf
    finger_links = links2plot{f}; % indices for finger links
    if isempty(finger_links) % do not plot this finger
        continue;
    end
    map_finger = hand_map{f};
    
    if isa(map_finger,'struct') % palm is a struct. For fingers, it is cell array of links.
        if isequal(map_finger.idx, 0) % palm: link idx 0
            palmmesh = map_finger.linkmesh;
            scatter3(palmmesh(:,1),palmmesh(:,2),palmmesh(:,3),'MarkerFaceColor',cmap(cidx,:));
            hold on;
        else
            warning('Incorrect data type.');
        end
    else % map_finger: 1×5 cell array
        %%% plot rmaps of finger links
        nlink = length(map_finger);
        for l = 1:nlink-1 % number of real links (ignore fingertip)
            if ismember(l,finger_links) || any(isnan(finger_links))
                % If the index of link, l, is in the list of desired link to plot;
                % or if map_idx_f = NaN, the entire finger should be plotted
                map_link = map_finger{l};
                
                if isfield(map_link,'p') && isfield(map_link,'linkmesh') % This is for Allegro hand model used in 'inhand_exploration'
                    map_dataset = cat(1, map_link.p, map_link.linkmesh); % p: samples of finger digit base; linkmesh: samples of finger digit tip
                else
                    k = map_link.bndryIndices; % Boundary set of joint reachable space, one CH for each finger
                    k_ch = map_link.cnvxIndices;
                    map_dataset = map_link.linkmesh;
                end

                %{
                % Add gaussian noise to avoid singularity in convexhull
                noise_sigma = 1;
                noise_mu = 0;
                noise = dataset + sqrt(noise_sigma)*randn(size(dataset)) + noise_mu;
                dataset = cat(1, dataset, noise);
                %}
                px = map_dataset(:,1); % optional: down-sampling to plot less
                py = map_dataset(:,2);
                pz = map_dataset(:,3);

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Option I: plot original data
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %{
                if isempty(k) || isempty(k_ch) || isvector(k) || isvector(k_ch)
                    % incase of degenerated convex hull
                    scatter3(px,py,pz,'MarkerFaceColor',cmap(cidx,:));
                end
                %}
                %{
                scatter3(px,py,pz,'MarkerFaceColor',cmap(cidx,:));
                cidx = cidx + 1; % for generating color
                hold on;
                %}

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Option II: convex hull
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %{
                [k_noisy,~] = boundary(px,py,pz,0); % 0: convex hull; 1: compact boundary that envelops the points. default: 0.5
                idx = idx + 1; % for generating color
                trisurf(k_noisy,px,py,pz,'Facecolor',cmap(idx,:),'EdgeColor','none');
                hold on;
                %}
                
                %{
                trisurf(k_ch,px,py,pz,'Facecolor',cmap(cidx,:),'EdgeColor','none','FaceAlpha',0.5); % convex hull
                cidx = cidx + 1; % for generating color
                hold on;
                %}

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Option III: alpha shape
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                shp = alphaShape(px,py,pz,35); % Inf: produces the convex hull
                plot(shp,'Facecolor',cmap(cidx,:),'EdgeColor','none','EdgeAlpha',0.25,'FaceAlpha',0.35);
                cidx = cidx + 1; % for generating color
                hold on;
                
                % if isempty(k) && isempty(k_ch)
                %     scatter3(px,py,pz,'MarkerFaceColor','none','MarkerFaceColor','k'); hold on; % Scatter all sampled points
                % else
                %     trisurf(k,px,py,pz,'Facecolor',cmap(i,:),'EdgeColor','none'); hold on;
                % end

                %%% [Vector Field] Visualize the norm directions of fingertip
                % vecNorm = map_link.n; % finger belly normal vector
                % vn_x = vecNorm(:,1); % x coordinate of vector
                % vn_y = vecNorm(:,2); % y coordinate of vector
                % vn_z = vecNorm(:,3); % z coordinate of vector
                % vn_x = vn_x(1:2:end,:); % normal vector pointing to the corresponding direction
                % vn_y = vn_y(1:2:end,:);
                % vn_z = vn_z(1:2:end,:);
                % cl = [0, 0.4470, 0.7410]; % red color
                % quiver3(px,py,pz,vn_x,vn_y,vn_z,'Color',cl,'MaxHeadSize',0.5,'AutoScale','on','AutoScaleFactor',2);
            % else
                % fprintf('Skip plotting: F%dL%d\n', f, l);
            end
        end
    end
end

grid on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');

view([-150 30]);

end