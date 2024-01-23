function [min_norm, min_idx] = minErrorRowTransformation(Y,X)
    % Find the corresponding list of index, so that Y - X(:,idx) has the
    % minimum norm.
    % X: (3,N), N sampled points, source
    % Y: (3,N), N sampled points, target
    
    if size(X,1) > size(X,2)
        X = transpose(X);
    end
    if ~isequal(size(X),size(Y))
        if isequal(size(X),size(transpose(Y)))
            Y = transpose(Y);
        else
            error('Inconsistent X and Y dimensions.');
        end
    end
    
    Ns = size(X,2); % number of samples

    testList = perms(1:Ns);
    Np = length(testList); % number of permutations
    dist = zeros(1,Np); % total distance for each permutation

    for i = 1:Np
        iperm = testList(i,:);
        E = Y - X(:,iperm); % element-error matrix
        dist(i) = norm(E);
    end

    [min_norm, idx] = min(dist);
    min_idx = testList(idx,:);
end