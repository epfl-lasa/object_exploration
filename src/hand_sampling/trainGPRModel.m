function [Model, y_star] = trainGPRModel(X,y)
    % This model is the explicit expression of y*
    % X: (N,4), y: (N,1)
    
    assert(size(X,1)==size(y,1));
    N = length(y); % number of samples
    d = size(X,2); % dimension
    
    %% Train GPR model using MATLAB built-in function, as baseline
    Model = fitrgp(X,y,'Basis','none'); % important to set 'Basis' to 'none'
    
    % Squared Exponential Kernel
    SigmaL = Model.KernelInformation.KernelParameters(1);
    SigmaF = Model.KernelInformation.KernelParameters(2);
    
    Sigma = Model.Sigma; % noise
    
    K = constructKMatrix(X,X,SigmaL,SigmaF); % (N,N)
    % Use symbolic model for inference
    X_star = sym('x',[1,d]);
    K_star = constructKMatrix(X,X_star,SigmaL,SigmaF); % (N,1)
    
    y_star = (K + Sigma^2*eye(N))\y; % Split calculation to avoid calculating symbolic expression in matrix inverse
    y_star = transpose(K_star) * y_star; % (1,1)
    
    %{
    gprMdl = fitrgp(X,y);
    ypred = resubPredict(gprMdl); % predict responses for training observations
    ypred = predict(gprMdl,Xnew);
    %}
    disp('trainGPRModel: completed.');
    
end


function K = constructKMatrix(X1,X2,SigmaL,SigmaF)
    % X1: (N1,:), data
    % X2: (N2,:), data or inference sample
    assert(size(X1,1)>size(X1,2));
    
    N1 = size(X1,1); % number of samples for X1
    N2 = size(X2,1); % number of samples for X2
    
    if isa(X1,'sym') || isa(X2,'sym')
        K = sym(zeros(N1,N2));
    else
        K = zeros(N1,N2);
    end
    fprintf('Training dimensionality: [%d,%d]\n',N1,N2);
    n = 0;
    % n_total = N1*N2;
    for i = 1:N1
        for j = 1:N2
            n = n + 1;
            % fprintf('%d - %d\n',n_total,n);
            K(i,j) = kernelFunction(X1(i,:),X2(j,:),SigmaL,SigmaF);
        end
    end
end


function k = kernelFunction(x1,x2,SigmaL,SigmaF)
    % Squared Exponential Kernel
    % ref: https://ch.mathworks.com/help/stats/kernel-covariance-function-options.html
    % x1, x2: (1,:)
    k = SigmaF^2 * exp(-(x1-x2)*transpose(x1-x2)/(2*SigmaL^2));
end