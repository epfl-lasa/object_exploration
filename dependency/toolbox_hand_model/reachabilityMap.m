% To test this module, just run this file w/o any parameters.

function [map_hand, robot] = reachabilityMap(robot, fngrs2spl, if_plot, if_save)
% Calculate the rechability map of the given robotic hand. Reachability map is a convex set of the spatial positions that
% the finger patches (usually finger tips) of the robotic hand can reach.
% Input:
%     * robot: robot model. if not assigned, use paradigmatic hand instead
%     * f_spl: list of boolean, indicate finger to sample, e.g. [1,0,0,0]. if not assigned, sample all fingers of the robot
%     % * if_plot: if plot the sampled reachability map [moved to function 'plotReachabilityMap']
%     * if_save: if save the sampled reachability map
% Output:
%     * map_hand: (1,nf) cell, contains the reachable space of each finger
%     * robot: robot structure, the map of each link has been updated in
%     the robot model
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if nargin < 1 % set default robotic hand model
        close all;
        clear all;
%         robot = mySGparadigmatic();
        robot = mySGallegroLeft();
    end
    nf = robot.n; % number of fingers to sample
    if nargin < 2 || isequal(fngrs2spl,'all') % idx of fingers to sample, e.g. [0,1,0,0,0] to sample only index finger for a paradigmatic hand model
        fngrs2spl = ones(1,nf); % [1,1,1,1,1], sample all
    end
    if nargin < 3
        if_plot = true; % if visualize the reachability map after constructing it
    end
    if nargin < 4
        if_save = true; % determine if save the results to mat file
    end
    
    if ~isfield(robot, 'reachability_map') % check if the robot already has reachability map
        robot.reachability_map = false;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    map_hand = cell(1,robot.n+1); % the (nf+1)th rmap is the map of palm
    for f = 1:nf % iterate over each finger
        
        % disp('--------------------------------------------------');
        fprintf('Sampling finger: %d\n', f);
        if (~fngrs2spl(f))
            fprintf('Finger %d is undesired to sample, skipped.\n', f);
            continue;
        end
        if (~robot.factv(f))
            fprintf('Finger %d is inactive, skipped.\n', f);
            continue;
        end
        delta_q = deg2rad(10); % sampling step length (joint angle, q, in degree) to radius

        finger = robot.F{f}; % current finger
        nl = finger.nlink; % number of links of f-th finger
        nd = finger.n; % number of dof for f-th finger (1 fewer than nl, as nl contains fingertip)
        
        map_finger = cell(1, nl); % construct the rmap of the finger, treat fingertip as one link
        
        % Generate meshgrid for sampling over joint space, instead of using multi-later for-loops
        q_dim = cell(1,nd); % Input variables: each dimension (joint) of q
        for d = 1:nd
            if ~finger.active_joints(d) % if already used (inactive)
                fprintf('Joint %d is not active.\n', d);
                q_dim{d} = finger.q(d); % keep the current angle
            else
                q_dim{d} = finger.lb(d):delta_q:finger.ub(d); % sample from lower bound to upper bound
            end
            delta_q = delta_q * 1.2; % attenuation factor, increase the sample step length
        end
        q_mesh = cell(1,nd); % output meshgrid, each cell is value of one dimension
        [q_mesh{:}] = ndgrid(q_dim{:});
        num_q = numel(q_mesh{1}); % number of all q-values to sample
        % fprintf('Total number of joint angle combinations: %d\n', num_q);
        
        for idx = 1:num_q % iterate over all possible joint-value-combinations
            q_test = zeros(1,nd); % current q to sample
            for d = 1:nd % assign each joint value
                t = q_mesh{d}; % t: test
                t = t(:);
                q_test(d) = t(idx);
            end
            
            finger = moveFinger(finger, q_test, 'sym'); % link information is updated in 'moveFinger' function
            % finger = moveFinger(finger, q_test, 'num'); % Should be the same (for test purpose)
            
            for l = 1:finger.nlink % iterate over links to step-by-step construct the link base point (fingertip is treated as the last link)
                link = finger.Link{l}; % current link
                map_link = map_finger{l}; % reachability map for link{l}
                if isempty(map_link) % to initialize the link-reachability-map
                    map_link.p = [];
                    map_link.r = [];
                    map_link.n = [];
                end
                map_link.p = cat(1, map_link.p, transpose(link.p(:))); % (N,3), NOTICE that this is link base position
                map_link.r = cat(1, map_link.r, transpose(link.r(:))); % (N,3), vector of radial direction (fixed w.r.t. link base)
                map_link.n = cat(1, map_link.n, transpose(link.n(:))); % (N,3), vector of normal direction of the link base (fixed w.r.t. link base)
                map_finger{l} = map_link; % update
                
            end
            
            if ~mod(idx, floor(num_q/10)) % print after completing every 1/10 samples
                % fprintf(' %d - %d\n', num_q, idx)
                fprintf(' %d%% ', 10*floor(idx/floor(num_q/10)));
                if floor(idx/floor(num_q/10)) == 10
                    fprintf('\n');
                end
            end
            
        end
        
        %% Remove repetitive entries in the fields of map_link
        for l = 1:length(map_finger)
            % disp('Remove repetitive samples...');
            map_link = map_finger{l};
            [~, ia] = unique(map_link.p, 'rows'); % (N,3), keep only the unique values
            map_link.p = map_link.p(ia,:); % (N,3), keep only the unique values
            map_link.r = map_link.r(ia,:); % (N,3), keep only the unique values
            map_link.n = map_link.n(ia,:); % (N,3), keep only the unique values
            map_finger{l} = map_link;
        end
        
        %% Construct meshgrid of link based on sampling of finger joints (obtained above) and then construct convex hull of sample points for each link (instead of each finger)
        for l = 1:finger.nlink-1 % Only for all real and virtual links except fingertip
            % disp('Construct link meshgied...');
            map_link = map_finger{l};
            map_next = map_finger{l+1};
            linkmesh = cat(1, map_link.p, map_next.p); % concatenate the samples of the link start (this) and link end (next)
            linkmesh = unique(linkmesh, 'rows'); % (N,3), keep only the unique values
            
            try % in case dataset degenerates and convex hull does not exist
                [k,~] = boundary(linkmesh); % Use boundary, not convex hull. Set last input to 0 to use convex hull. default: 0.5
                [k_ch,~] = convhulln(linkmesh); % Use convex hull
            catch % k or k_ch is empty
                if size(linkmesh,1) > 2 % a 2d plane exists in 3d space
                    [linkmesh,k,k_ch] = mesh2DPlane(linkmesh); % generate meshgrid for degenerated plane
                else
                    k = [];
                    k_ch = [];
                end
            end
            
            if (finger.idx_real_link(l) && isempty(k)) || (~finger.idx_real_link(l)) % only for real link
                fprintf('Finger %d Link %d does not have convexhull.\n', f, l);
                fprintf('* isempty(k): %d\n', isempty(k));
                fprintf('* real link: %d\n', finger.idx_real_link(l));
            end
            
            map_link.linkmesh = linkmesh; % the meshgrid of all reachable positions of the link
            map_link.bndryIndices = k; % Boundary set of joint reachable space, one CH for each finger
            map_link.cnvxIndices = k_ch; % Indices of points that form the Convex set
            
            map_finger{l} = map_link; % assign value to finger reachability map
            
            if nargout > 1
                robot.reachability_map = true; % flag to indicate that all rmaps have been added to link field
                robot.F{f}.Link{l}.map_link = map_link; % update robot model by adding the rmap to the link
            end
        end
        map_hand{f} = map_finger; % update the link of the hand
    end
    
    % disp('--------------------------------------------------');
    % disp('Sampling on palm surface: map_palm');
    % Samples on palm inner surface
    % Reference: Sulimon Sattari (2020). Grid of points within a polygon (https://www.mathworks.com/matlabcentral/fileexchange/41454-grid-of-points-within-a-polygon), MATLAB Central File Exchange. Retrieved September 26, 2020.
    % Theorem: projection of convex hull = convex hull of projection
    
    basepoints_xy = robot.T\robot.P.basepoints_h; % (4,N), project palm vertices back on XY plane (before multiply T)
    ppa = 1.5*1.5; % point per unit area (1x1)
    palmmesh_xy = polygrid(basepoints_xy(1,:).', basepoints_xy(2,:).', ppa); % (K,2) 100 points inside
    palmmesh_xyz = cat(2,palmmesh_xy,ones(size(palmmesh_xy,1),1)); % (N,3) add Z = 0 coordinates
    palmmesh_xyz_h = cat(2,palmmesh_xyz,ones(size(palmmesh_xyz,1),1)); % (N,4), homogeneous coordinates

    palmmesh_h = robot.P.HT2inr*(robot.T*palmmesh_xyz_h.'); % first project back to palm plane, then project from palm to palm inner surface
    palmmesh = palmmesh_h(1:3,:).'; % (N,3)
    map_hand{nf+1} = struct('idx', 0, 'linkmesh', palmmesh);
    % disp('--------------------------------------------------');
    
    if if_plot
        plotReachabilityMap(robot, map_hand);
        % plotReachabilityMap(robot, map_hand, {[],[1,2,3,4,5],[],[],[]});
    end

    if if_save
        save('../database/reachable_map.mat','map_hand');
        disp('File saved: reachable_map.mat');
    end
end