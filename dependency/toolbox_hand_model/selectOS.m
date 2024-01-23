function osCandList = selectOS(OS, besetztOS, n_cand)
% This function selects a list of opposition space candidates from all os set, OS.

if nargin<3
    n_cand = 1; % number of desired os candidates
end

if nargin<2 % [Only for Single Grasp]
    osCandList = selectSingleOS(OS);
else
    %%% Step 1: remove besetzt_os from the OS list
    availOS = {};
    costOS = []; % to save the cost of using this OS for grasping
    for i = 1:numel(OS)
        os_info = OS{i}.os_info;

        still_avail = true; % boolean variable to mark if the os is still available
        for j = 1:numel(besetztOS)
            used_os_info = besetztOS{j}.os_info; % desired pair, in [F,L] order
            
            %%% If one OS has been used then remove it from current
            %%% available list
            if (isequal(os_info{1},used_os_info{1}) && isequal(os_info{2},used_os_info{2})) ||...
                    (isequal(os_info{1},used_os_info{2}) && isequal(os_info{2},used_os_info{1})) % symmetric case
                % this pair of OS does not exit in besetzt os list 
                still_avail = false;
            end

            %{
            %%% Strict rules: if one link has been used then remove it from all available OSs
            if isequal(os_info{1},used_os_info{1}) || isequal(os_info{2},used_os_info{2}) ||...
                    isequal(os_info{1},used_os_info{2}) || isequal(os_info{2},used_os_info{1}) % symmetric case
                still_avail = false;
            end
            %}
        end
        if still_avail
            availOS{end+1} = OS{i};
            costOS(end+1) = max(OS{i}.os_dist); % use the max. distance of the OS as the cost
        end
    end
    % fprintf('Total OS: %d, used OS: %d, avaliable OS: %d\n',numel(OS),numel(besetztOS),numel(availOS));

    %%% Step 2: select one (or a few) OS out of all available OSs, based on some metric
    [~,min_idx] = sort(costOS); % in ascending order
    osCandList = availOS(min_idx(1:n_cand)); % extract the first n_cand OSs

    if isempty(osCandList)
        warning('OS to fit the object does not exist.');
    end
    % fprintf('Selected %d out of %d OS for testing.\n',numel(osCandList),numel(availOS));
end

end