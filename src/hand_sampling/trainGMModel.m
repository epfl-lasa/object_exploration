function Model = trainGMModel(X,y,nK)
    % X: (N,4), y: (N,1)
    % nK: largest number of K to test
    assert(size(X,1)==size(y,1));
    data = cat(2,X,y);
    
    if nargin < 3
        nK = 10;
    end
    
    AIC = zeros(1,nK);
    BIC = zeros(1,nK);
    GMModels = cell(1,nK);
    
    options = statset('MaxIter',1000);
    for k = 1:nK
        % fprintf('k: %d\n',k);
        try
            GMModels{k} = fitgmdist(data,k,'Options',options,'CovarianceType','diagonal','Replicates',1);
            AIC(k) = GMModels{k}.AIC;
            BIC(k) = GMModels{k}.BIC;
        catch % In case of ill-conditioned covariance
            GMModels{k} = [];
            AIC(k) = Inf;
            BIC(k) = Inf;
        end
    end
    
    %{
    figure, hold on;
    
    subplot(1,3,1);
    plot(AIC);
    xlabel('K');
    title('AIC');
    
    subplot(1,3,2);
    plot(BIC);
    xlabel('K');
    title('BIC');
    
    subplot(1,3,3);
    plot(gradient(BIC));
    xlabel('K');
    title('BIC gradient');
    
    sgtitle('Selection of optimal number of components');
    hold off;
    %}
    
%     [~,idxAIC] = min(AIC);
    [~,idxBIC] = min(BIC);
    
    fprintf('The optimal number of components based on BIC is %d\n', idxBIC);
    
    Model = GMModels{idxBIC}; % select best model based on BIC index
end
