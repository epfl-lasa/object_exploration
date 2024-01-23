function V = symCross(X,Y)
	% X, Y should be either vectors (3,1) or (1,3), or matrices (3,N). 
	% Not necessarily symbolic expressions, works with numerical expressions as well.
    
    assert(isequal(size(X),size(Y)));
    
    if isrow(X)
        X = X.';
    end
    if isrow(Y)
        Y = Y.';
    end

	V = [X(2,:).*Y(3,:) - X(3,:).*Y(2,:);...
	X(3,:).*Y(1,:) - X(1,:).*Y(3,:);...
	X(1,:).*Y(2,:) - X(2,:).*Y(1,:)];

end