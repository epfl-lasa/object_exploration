function osCandList = selectSingleOS(OS, tgt_pair)

    if nargin < 2
        warning('desired os pair not given.');
        tgt_pair = {[1,4],[2,4]}; % desired pair
    end

    osCandList = {}; % list of testing os pairs in Opposition Space
    for i = 1:numel(OS) % around 90 Opposition Space in total
        os_info = OS{i}.os_info;

        if (isequal(os_info{1},tgt_pair{1}) && isequal(os_info{2},tgt_pair{2})) ||...
            (isequal(os_info{1},tgt_pair{2}) && isequal(os_info{2},tgt_pair{1})) % symmetric
            osCandList{end+1} = OS{i}; % this ith os_info exists in the testing pair, so register the index
        end
    end

    if isempty(osCandList)
        warning('OS to fit the object does not exist.');
    end
end