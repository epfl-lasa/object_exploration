% Remove the points in `ptCloud` that are closest to `Samples`

function ptCloud = removeFromCloud(ptCloud, Samples)
    % ptCloud: (N,3)
    % subSet: (N,3)
    
    if size(ptCloud,1) == 3
        ptCloud = transpose(ptCloud);
    end
    if size(Samples, 1) == 3
        Samples = transpose(Samples);
    end
    
    % Alternative: ismember, ismembertol [Not robust]
    
    % nClouds = size(ptClouds, 1); % (S,R)
    nSamples = size(Samples, 1);
            
    for iS = 1:nSamples
        sample = Samples(iS,:);
        sample = reshape(sample,3,1);
        distList = dist(ptCloud, sample); % S: nClouds, R: 3, Q: 1
        [~,idx] = min(distList);
        ptCloud(idx,:) = [];
    end
end