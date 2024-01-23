% This function generates next step samples based on the current samples
% and step width

function [next_contacts] = generateNextStepSamples(hand, this_contacts, objectCloud, vecT, vecN)
    % vecT: tangential direction of each contact point
    % vecN: surface normal of each contact point

    % next_reference_positions: directly calculated reference target points using vecT
    % next_contacts: projected next_reference_positions on the object point cloud, made sure to lie on the object surface
    
    % global Hz
    global explorationStep d_epsilon
    global reassignTargetsForFingers
    global projectNextContacts % true if using MATLAB only; false if using MuJoCo

    if nargin < 4
        vecN = zeros(3,4); % Stay at current position
    end
    if nargin < 3
        vecT = zeros(3,4); % Stay at current position
    end
    if size(vecN,2) == 3
        vecN = transpose(vecN);
    end
    
    if size(vecT,2) == 3
        vecT = transpose(vecT);
    end

    this_positions = this_contacts.positions; % (3,4), current contacts of each finger
    this_normals = this_contacts.normals;
    
    %% Calculate desired exploration position
    % Temporary solution: using linear fit (ignore normal direction)
    next_reference_positions = this_positions + vecT * explorationStep; % (3,4) step length: 2cm, virtual targets
    next_normals = vecN; % [todo] Keep vecN as next normals for now
    
    %% Find REAL contacts: find the point cloud that closest to the fingertip
    if projectNextContacts
        real_positions = zeros(size(next_reference_positions)); % (3,N)
        for i = 1:size(next_reference_positions,2)
            finger_next_pos = next_reference_positions(:,i); % (3,1), the desired next position for finger iF
            
            %%% Approach 1: filtering out the neighbours to avoid repetition
            % distMtx = dist(objectCloud, finger_next_pos); % (N,1)
            % d_epsilon = 10; % 1cm
            % distMtx_epsilon = distMtx(distMtx>=d_epsilon); % remove the points inside the epsilon closure, to avoid exploring the neighbour points
            % idx = find(distMtx == min(distMtx_epsilon));
            
            %%% Approach 2: no filtering of neighbours
            distList = pdist2(finger_next_pos', objectCloud);
            [~, idx] = min(distList);
            
            real_positions(:,i) = objectCloud(idx,:); % (3,4) real-existing points on object surface
            % objectCloud(idx,:) = [];
        end
        
        %% Mapping approach 5: minimize the total moving distance (from current fingertips to targets)
        %%% Step 1: assign targets to each fingertip
        [~, idx_np] = minErrorRowTransformation(this_positions, real_positions); % input: (3,N)
        next_positions = real_positions;
    else
        idx_np = 1:4;
        next_positions = next_reference_positions;
    end

    next_positions = next_positions(:,idx_np);
    next_normals = next_normals(:,idx_np);
    
    %%% Step 2: mapping targets for 2-3-4 fingers to further avoid collision, project to F2-F4 line
    if reassignTargetsForFingers
        real_positions_fingers = next_positions(:,2:4); % (3,N-1), exclude thumb
        real_normals_fingers = next_normals(:,2:4);
        
        Fbase = zeros(3,4); % (3,N)
        % Fbase(:,1) = hand.F{1}.base(1:3,4); % (3,1)
        Fbase(:,2) = hand.F{2}.base(1:3,4);
        % Fbase(:,3) = hand.F{3}.base(1:3,4);
        Fbase(:,4) = hand.F{4}.base(1:3,4);

        F2F4 = Fbase(:,4) - Fbase(:,2); % (3,1)
        p_ref = Fbase(:,2) - 1e3 * F2F4/norm(F2F4); % (3,1), reference point, lie to the left side of F2 base
        p_axis = Fbase(:,4) - p_ref; % (3,1), establish the axis of baseline: from p_ref (left side) to Fbase(:,4) (right side)

        mapped_contacts = real_positions_fingers + p_axis / dot(p_axis,p_axis) * dot(real_positions_fingers-p_ref, repmat(p_axis,1,size(real_positions_fingers,2))); % use formula of projecting P to AB as P': P' = A + dot(AP,AB)/dot(AB,AB) * AB
        
        % For debugging
        dist_vec = bsxfun(@minus, mapped_contacts, p_ref);
        dist_squared = sqrt(sum(dist_vec.*dist_vec));
        [~, idx_fingers_test] = sort(dist_squared,'ascend'); % distance closest to p_ref
        
        % Mapping real_positions_fingers to mapped_contacts (on hand finger base)
        [~, idx_fingers] = minErrorRowTransformation(mapped_contacts, real_positions_fingers);
        
        if ~isequal(idx_fingers_test, idx_fingers)
            warning('Inconsistent sorting indices.');
            fprintf('* Mapping based on orders of projections: %d, %d, %d, %d\n',...
                idx_fingers_test(1), idx_fingers_test(2), idx_fingers_test(3), idx_fingers_test(4));
            fprintf('* Mapping based on min. distance to projected points: %d, %d, %d, %d\n',...
                idx_fingers(1), idx_fingers(2), idx_fingers(3), idx_fingers(4));
        end
        
        real_positions_fingers = real_positions_fingers(:,idx_fingers);
        real_normals_fingers = real_normals_fingers(:,idx_fingers);

        next_positions(:,2:4) = real_positions_fingers;
        next_normals(:,2:4) = real_normals_fingers;

        idx_np_subset = idx_np(2:4);
        idx_np(2:4) = idx_np_subset(idx_fingers);
    end
    
    %% Mapping approach 1: re-map the calculated next positions to the targets of each fingertip according to X coordinates
    %{
    [~, idx_np] = sort(real_positions(1,:), 'descend'); % sort along the X dimension of the hand reference frame
    % if ~isequal(idx_np, 1:4)
    %     fprintf('Sorted indices: %d, %d, %d, %d\n', idx_np(1), idx_np(2), idx_np(3), idx_np(4));
    % end
    next_positions = real_positions(:,idx_np);
    next_normals = next_normals(:,idx_np);
    %}

    
    %% Mapping approach 2: projecting target points to the baselines of fingers
    %{
    %%% First mapping: map all desired points to the line F1-F4 (thumb - ring finger), to find the desired contact of the thumb
    Fbase = zeros(3,4);
    Fbase(:,1) = hand.F{1}.base(1:3,4);
    Fbase(:,2) = hand.F{2}.base(1:3,4);
    Fbase(:,3) = hand.F{3}.base(1:3,4);
    Fbase(:,4) = hand.F{4}.base(1:3,4);

    F1F4 = Fbase(:,4) - Fbase(:,1); % (3,1), Line F1 -> F4
    p_ref = Fbase(:,1) - 1e3 * F1F4/norm(F1F4); % p_ref is a point to the very left side of F1
    p_axis = Fbase(:,4) - p_ref; % (3,1), projection line p_ref -> F4

    mapped_contacts = real_positions + p_axis / dot(p_axis,p_axis) * dot(real_positions-p_ref, repmat(p_axis,1,4)); % use formula of projecting P to AB as P': P' = A + dot(AP,AB)/dot(AB,AB) * AB
    dist_vec = bsxfun(@minus, mapped_contacts, p_ref);
    dist_squared = sqrt(sum(dist_vec.*dist_vec));
    [~,idx_thumb] = min(dist_squared);

    next_thumb = real_positions(:,idx_thumb);
    assert(isequal(size(next_thumb),[3,1]));

    real_positions_exclude_thumb = real_positions;
    real_positions_exclude_thumb(:,ismember(real_positions',next_thumb','rows')) = []; % (3,N=3) remove next_contact_thumb from the set of contacts

    %%% Second mapping: map all remaining points to the line F2-F4, to find the desired contact for index, middle, and ring finegr
    F2F4 = Fbase(:,4) - Fbase(:,2);
    p_ref = Fbase(:,2) - 1e3 * F2F4/norm(F2F4);
    p_axis = Fbase(:,4) - p_ref;

    mapped_contacts = real_positions_exclude_thumb + p_axis / dot(p_axis,p_axis) * dot(real_positions_exclude_thumb-p_ref, repmat(p_axis,1,3)); % use formula of projecting P to AB as P': P' = A + dot(AP,AB)/dot(AB,AB) * AB
    dist_vec = bsxfun(@minus, mapped_contacts, p_ref);
    dist_squared = sqrt(sum(dist_vec.*dist_vec));
    [~,idx] = sort(dist_squared,'ascend');

    next_index = real_positions_exclude_thumb(:,idx(1));
    next_middle = real_positions_exclude_thumb(:,idx(2));
    next_ring = real_positions_exclude_thumb(:,idx(3));

    %%% Check the order in original real_positions
    idx_np = [find(ismember(real_positions',next_thumb','rows')),...
    find(ismember(real_positions',next_index','rows')),...
    find(ismember(real_positions',next_middle','rows')),...
    find(ismember(real_positions',next_ring','rows'))];
    
    next_positions = [next_thumb,next_index,next_middle,next_ring];
    next_normals = next_normals(:,idx_np)
    %}
    
    
    %% Mapping approach 2-alternative-1: simplified version of M2, project to F1-F4 line [used in old version]
    %{
    Fbase = zeros(3,4);
    Fbase(:,1) = hand.F{1}.base(1:3,4);
    Fbase(:,2) = hand.F{2}.base(1:3,4);
    Fbase(:,3) = hand.F{3}.base(1:3,4);
    Fbase(:,4) = hand.F{4}.base(1:3,4);
    
    F1F4 = Fbase(:,4) - Fbase(:,1);
    p_ref = Fbase(:,1) - 1e3 * F1F4/norm(F1F4);
    p_axis = Fbase(:,4) - p_ref;

    mapped_contacts = real_positions + p_axis / dot(p_axis,p_axis) * dot(real_positions-p_ref, repmat(p_axis,1,4)); % use formula of projecting P to AB as P': P' = A + dot(AP,AB)/dot(AB,AB) * AB
    dist_vec = bsxfun(@minus, mapped_contacts, p_ref);
    dist_squared = sqrt(sum(dist_vec.*dist_vec));
    [~,idx_np] = sort(dist_squared,'ascend'); % distance closest to thumb

    next_positions = real_positions(:,idx_np);
    next_normals = next_normals(:,idx_np);
    %}
    
    
    %% Mapping approach 2-alternative-2: simplified version of M2, project to F2-F4 line
    %{
    Fbase = zeros(3,4);
    Fbase(:,1) = hand.F{1}.base(1:3,4);
    Fbase(:,2) = hand.F{2}.base(1:3,4);
    Fbase(:,3) = hand.F{3}.base(1:3,4);
    Fbase(:,4) = hand.F{4}.base(1:3,4);
    
    F2F4 = Fbase(:,4) - Fbase(:,2);
    p_ref = Fbase(:,2) - 1e3 * F2F4/norm(F2F4);
    p_axis = Fbase(:,4) - p_ref;

    mapped_contacts = real_positions + p_axis / dot(p_axis,p_axis) * dot(real_positions-p_ref, repmat(p_axis,1,4)); % use formula of projecting P to AB as P': P' = A + dot(AP,AB)/dot(AB,AB) * AB
    dist_vec = bsxfun(@minus, mapped_contacts, p_ref);
    dist_squared = sqrt(sum(dist_vec.*dist_vec));
    [~,idx_np] = sort(dist_squared,'ascend'); % distance closest to thumb

    next_positions = real_positions(:,idx_np);
    next_normals = next_normals(:,idx_np);
    %}
    
    
    %% Mapping approach 3: according to the min. distance
    %{
    backup_positions = real_positions;
    idx_np = zeros(1,4);
    % next_positions = zeros(3,4);
    
    for i = 1:4
        dist_vec = bsxfun(@minus, backup_positions, this_positions(:,i)); % this_positions(:,i) is the current contact of the ith finger
        dist_squared = sqrt(sum(dist_vec.*dist_vec));
        [~,idx] = min(dist_squared);
        % next_positions(:,i) = backup_positions(:,idx);
        idx_np(i) = find(ismember(real_positions',backup_positions(:,idx)','rows'));
        backup_positions(:,idx) = [];
    end
    
    next_positions = real_positions(:,idx_np);
    next_normals = next_normals(:,idx_np);
    %}
    
    
    %% Mapping approach 4: according to included angle
    %{
    Fbase = zeros(3,4);
    Fbase(:,1) = hand.F{1}.base(1:3,4);
    Fbase(:,2) = hand.F{2}.base(1:3,4);
    Fbase(:,3) = hand.F{3}.base(1:3,4);
    Fbase(:,4) = hand.F{4}.base(1:3,4);
    
    backup_positions = real_positions; % (3,4)
    idx_np = zeros(1,4);
    
    for iF = 1:4
        Tip = this_positions(:,iF); % this_positions(:,i) is the current contact of the ith finger
        base_tip = Tip - Fbase(:,iF); % line: finger base -> finger tip
        
        nc = size(backup_positions,2);
        included_angles = zeros(1,nc);
        for iC = 1:nc
            test_contact = backup_positions(:,iC);
            tip_contact = test_contact - Tip;
            
            included_angles(iC) = acos(dot(tip_contact,base_tip)/(norm(tip_contact)*norm(base_tip))); % cosine similarity
        end
        [~,idx] = min(included_angles);
        disp(included_angles);
        idx_np(iF) = find(ismember(real_positions',backup_positions(:,idx)','rows'));
        backup_positions(:,idx) = [];
    end
    
    next_positions = real_positions(:,idx_np);
    next_normals = next_normals(:,idx_np);
    %}
    
    if ~isequal(idx_np, 1:4)
        fprintf('Sorted indices: %d, %d, %d, %d\n', idx_np(1), idx_np(2), idx_np(3), idx_np(4));
    end
    
    next_contacts.positions = next_positions; % (3,4)
    next_contacts.normals = next_normals;
end