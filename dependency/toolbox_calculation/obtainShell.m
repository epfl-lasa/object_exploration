function pShell = obtainShell(P, kSet)
    % Obtain the "shell" of the boundary given the points P (N,3) and
    % triangulation indices k (Nk, 3) from MATLAB boundary function
    
    K = [];
    nd = size(kSet,2); % number of dimensions
    for d = 1:nd
        K = union(K,kSet(:,d)); % union of each dimension of k
    end
    pShell = P(K,:);
end