% Symbolic expression of MultiVariate Normal distribution pdf
function p = symMvnpdf(X, mu, Sigma)
    % X: (n,1), symbolic expression
    % mu: (n,1), Sigma, are given as constants
    if isvector(Sigma)
        Sigma = diag(Sigma);
    end
    X = X(:);
    mu = mu(:);
    n = length(X); % num of dimension
    p = exp(-(1/2)*transpose(X-mu)/Sigma*(X-mu))/sqrt(power(2*pi,n)*det(Sigma));
end