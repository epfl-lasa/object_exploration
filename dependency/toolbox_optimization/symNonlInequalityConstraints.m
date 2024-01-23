% Obtain the symbolic form of nonlinear inequality constraints
function [c, c_grad, param, ht_c, ht_c_grad] = symNonlInequalityConstraints(hand, param)
    os_info = param.os.os_info;
    
    obj_r = param.object_radius; % object radius
    link_r = param.hand_radius;
    
    ncp = param.ncp; % number of contacts
    X_key = param.X_key;
    oc = [sym('x');sym('y');sym('z')]; % object center
    dict_q = param.dict_q; % dictionary of symbolic variables: q11,q12,...
    
    fprintf('\nConstructing Nonl. Inequality Constraints: \n');
    c = sym([]); % to save the symbolic form
    c_idx = []; % to save idx of different constraints
    c_name = {};
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Inequality Constraint 1: Collision Avoidance
    % Collision between links (this link and all father links) and object
    % dist. between all previous links & object larger than object radius
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Notice that this only contains all previous links (incl. virtual
    % ones), since the current link is in contact, and this constraint is
    % formulated as nonl equality constraint.
    fprintf('* Collision avoidance (object vs. links): ');
    for i = 1:ncp
        if ~all(os_info{i})
            % collision with palm is guaranteed in 'Collision avoidance (object vs. palm)' 
            continue;
        else
            [idx_f, idx_l] = deal(os_info{i}(1),os_info{i}(2)); % index of finger and link
            finger = hand.F{idx_f};
            nq_pre = min([idx_l, finger.n]); % number of joints ahead of idx_lnk, in case idx_lnk > finger.n (possible for fingertip link)
            for j = 1:nq_pre % Iterate over all links, including link in contact ('idx_l')
                link = finger.Link{j};
                if ~link.is_real
                    continue;
                end
                link_dist = link.symbolic.link_dist;
                c_ij = -(link_dist-link_r-obj_r); % symvar: [q1...q4, x, y, z]
                c(end+1) = c_ij; % c<=0; distance from link to should larger than radius of finger digit cylinder plus radius of object
            end
        end
    end
    c_idx(end+1) = numel(c);
    c_name{end+1} = 'Collision avoidance (object vs. links)';
    fprintf('%d\n', c_idx(end));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Inequality Constraint 2: Collision Avoidance
    % Collision between object and included fingers
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fprintf('* Collision avoidance (object vs. included fingers): ');
    % included finger: finger that between two active fingers, e.g. the middle finger is included if using index finger and ring finger for grasping
    f_idx_list = zeros(1,ncp); % list of finger index
    for i = 1:ncp
        f_idx_list(i) = os_info{i}(1);
    end
    f_idx_list(~f_idx_list) = []; % remove 0 (palm) from list
    n_diff = max(f_idx_list)-min(f_idx_list); % number of in-between fingers
    
    if n_diff>1 % in-between finger exists
        for f = min(f_idx_list)+1:max(f_idx_list)-1
            finger = hand.F{f}; % same as collision avoidance: father-links vs. object
            for j = 1:finger.n-1
                link = finger.Link{j};
                link_dist = link.symbolic.link_dist; % this is the distance from link central axis to an external 3d point [x,y,z]
                c_ij = -(link_dist-link_r-obj_r); % symvar: [q1...q4, x, y, z]
                c_ij = subs(c_ij, finger.q_sym, finger.q.');
                c(end+1) = c_ij;
            end
        end
    end
    
    c_idx(end+1) = numel(c)-sum(c_idx);
    c_name{end+1} = 'Collision avoidance (object vs. in-between fingers)';
    fprintf('%d\n', c_idx(end));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Inequality Constraint 4: Collision Avoidance
    % Collision between object and palm (finger bases)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fprintf('* Collision avoidance (object vs. palm): ');
    
    x1 = hand.F{1}.symbolic.base(1:3,4);
    x2 = hand.F{end}.symbolic.base(1:3,4);
    % Line x1-x2 is the 'finger base line' that connects all finger bases
    % This detection is realized by calculating the distance between object
    % center and the finger base line
    dist = norm(cross(oc-x1,oc-x2))/norm(x2-x1);
    c(end+1) = link_r + obj_r - dist;
    
    c_idx(end+1) = numel(c)-sum(c_idx);
    c_name{end+1} = 'Collision avoidance (object vs. palm)';
    fprintf('%d\n', c_idx(end));

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Inequality Constraint 3: Collision Avoidance
    % Collision between current link and links from other fingers
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fprintf('* Collision avoidance (this link vs. other links): ');
    all_collision_pairs = {};
    for i = 1:ncp
        if ~all(os_info{i})
            continue; % palm does not move, will not collide with other fingers
        else
            [idx_f,idx_l] = deal(os_info{i}(1),os_info{i}(2)); % index of finger and link in the finger
            finger = hand.F{idx_f};
            nq_pre = min([idx_l, finger.nlink-1]); % number of joints ahead of idx_lnk, in case idx_lnk > finger.n (possible for fingertip link). max(finger.n) = 4

            for j = 1:nq_pre % Iterate over the current link + all father-links on this finger
                link = finger.Link{j}; % link j, The link being checked at this moment
                if ~link.is_real
                    continue;
                end
                x1 = link.symbolic.HT_this(1:3,4); % end points of current link
                x2 = link.symbolic.HT_next(1:3,4);
                coll = link.collList; % retrieve the links that collide with the current one

                for k = 1:numel(coll) % link k collides with current link
                    [k_f,k_l] = deal(coll{k}(1),coll{k}(2));

                    if k_f == idx_f % skip link on the same finger
                        continue;
                    end

                    link_coll = hand.F{k_f}.Link{k_l}; % link to collide with current link
                    if ~link_coll.is_real
                        continue;
                    end
                    
                    % filter out repetitive collision pairs 
                    already_exist = false; % if the current pair of collision already exists in the collision pair
                    if ~isempty(all_collision_pairs)
                        for p = 1:numel(all_collision_pairs)
                            temp_pair = all_collision_pairs{p}; % should be [2*2]: [finger_1, link_1; finger_2, link_2]
                            if ismember([idx_f,idx_l],temp_pair,'rows') && ismember([k_f,k_l],temp_pair,'rows')
                                already_exist = true;
                                break;
                            end   
                        end
                    end
                    if already_exist
                        continue;
                    else
                        all_collision_pairs{end+1} = [idx_f,idx_l;k_f,k_l];
                    end
                    y1 = link_coll.symbolic.HT_this(1:3,4); % end points of collided link
                    y2 = link_coll.symbolic.HT_next(1:3,4);
                    
                    % Here is the min. distance between two 3d line segments.
                    % Not min. distance between two 3d lines!
                    dist = distanceBTW2Lines(x1,x2,y1,y2,5); % minimum distance between two links (line segments) N=5: 5 sampling points
                    % All virtual father links will result in a 1*1 dist.
                    %%% Substitute all parameters (qi) that are not in X_key with hand joint values
                    symvars = symvar(dist);
                    subKeys = symvars(~ismember(symvars,X_key)); % symvars that belong to dict_q but not X_key, need to be substituted
                    if ~isempty(subKeys)
                        subValues = hand.q(ismember(dict_q,subKeys)); % subs them using hand default joint angle values
                        dist = subs(dist, subKeys, subValues.');
                    end
                    try
                        c(end+1 : end+numel(dist)) = 2*link_r - dist;
                    catch
                        disp('I don`t know.');
                    end
                end
                    
            end
        end
    end
    c_idx(end+1) = numel(c)-sum(c_idx);
    c_name{end+1} = 'Collision avoidance (this link vs. other links)';
    fprintf('%d\n', c_idx(end));

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Inequality Constraint 4: Collision Avoidance
    % Collision between target object and already grasped objects
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    grasped_objects = param.grasped_objects;
    if ~isempty(grasped_objects)
        fprintf('* Collision avoidance (target object and object in grasp): ');
        nobj = numel(grasped_objects);
        for i = 1:nobj
            obj_i = grasped_objects{i};
            oc_i = obj_i.center;
            radius_i = obj_i.radius;
            c(end+1) = (obj_r+radius_i) - norm(oc(:)-oc_i(:)); % distance between centers of target object and anyone of the grasped object is larger than the sum of their radius
        end
        c_idx(end+1) = numel(c)-sum(c_idx);
        c_name{end+1} = 'Collision avoidance (target object vs. grasped objects)';
        fprintf('%d\n', c_idx(end));
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Save symbolic form of constraints
    param.c_idx = c_idx;
    param.c_name = c_name;
    
    matlabFunction(c,'File','../database/symbolic_functions/nonl_c','Vars',X_key,'Optimize',false);

    %%% Calculate gradient Gradient follows the format. Notice that this is the TRANSPOSE of Jacobian!
    % [dc1/dx1, dc2/dx1;...
    %  dc1/dx2, dc2/dx2];
    c_grad = transpose(jacobian(c, X_key));
    
    matlabFunction(c_grad,'File','../database/symbolic_functions/nonl_c_grad','Vars',X_key,'Optimize',false);
    fprintf('  Total num. of nonl. inequality constraints: %d\n', numel(c));
    
    if nargout > 3 % create function handles
        ht_c = matlabFunction(c,'Vars',X_key);
        ht_c_grad = matlabFunction(c_grad,'Vars',X_key);
    end
end