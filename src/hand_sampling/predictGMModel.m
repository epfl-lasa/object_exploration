function y = predictGMModel(Model,X)
    % The definition of variables follows the convention on MATLAB documentation:
    % https://ch.mathworks.com/help/stats/gmdistribution.html#d123e444275
    % X: vector, (1,M-1)
    % y: scalar
    
%     if isa(X,'sym')
%         symExpression = true;
%     else
        symExpression = false;
%     end
    
    K = Model.NumComponents; % scalar
    M = Model.NumVariables; % scalar

    mu = Model.mu; % (K,M)
    Sigma = Model.Sigma; % (1,M,K), Sigma(1,:,i) contains the diagonal elements of the covariance matrix of component i.
    p = Model.ComponentProportion; % (1,K) % corresponds to pi_k, proportion of component k

    mu_X = mu(:,1:M-1); % (K,M-1)
    mu_y = mu(:,end); % (K,1) last element of training data is y
    
    Sigma_X = Sigma(1,1:M-1,:); % (1,M-1,K), all diagonal values
    % Sigma_y = Sigma(1,end,:); % (1,1,K)
    
    if symExpression
        hk = sym(zeros(1,K));
        yk = sym(zeros(1,K));
    else
        hk = zeros(1,K);
        yk = zeros(1,K);
    end
    
    for k = 1:K
        S_k = diag(Sigma(1,:,k)); % (M,M), all diagonal values of Sigma matrix
        S_k_XX = S_k(1:M-1, 1:M-1); % (M-1,M-1)
        % S_k_Xy = S_k(1:M-1, end); % (M-1,1), cov of (X,y)
        S_k_yX = S_k(1, 1:M-1); % (1,M-1)
        % S_k_yy = S_k(end, end); % (1,1)
        
        denominator = 0;
        for j = 1:K
            
            if symExpression
                denominator = denominator + p(j)*symMvnpdf(X,mu_X(j,:),Sigma_X(1,:,j));
            else
                denominator = denominator + p(j)*mvnpdf(X,mu_X(j,:),Sigma_X(1,:,j));
            end
            
            % p_num = mvnpdf(X, mu_X(j,:), Sigma_X(1,:,j)); % numerical pdf
            % p_sym = symMvnpdf(X, mu_X(j,:), Sigma_X(1,:,j)); % symbolic pdf
            % assert(abs(p_num-p_sym)<1e-6);
            
        end
        
        if symExpression
            hk(k) = (p(k)*symMvnpdf(X,mu_X(k,:),Sigma_X(1,:,k)))/denominator;
        else
            hk(k) = (p(k)*mvnpdf(X,mu_X(k,:),Sigma_X(1,:,k)))/denominator;
        end
        
        yk(k) = mu_y(k) + S_k_yX/S_k_XX * transpose(X-mu_X(k,:));
    end
    
    y = dot(hk,yk);
end