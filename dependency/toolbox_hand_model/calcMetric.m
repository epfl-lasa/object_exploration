function metric = calcMetric(finger, metric_type)
	% Calculate the metric 'metric_type' of finger 'F' configuration.
	% Input:
	% 	* finger: see 'moveFinger'
	% 	* metric_type: type of metric
	% Output:
	% 	* metric: calculated value of metric

    switch metric_type
        case 'manipulability'
            J = calcGeometricJacobian(finger);
            J = J(1:3,:); % only sub-jacobian for linear velocity
            [~,S,~] = svd(J,'econ'); % V*D*W' = JF
            omega = prod(diag(S));
            metric = omega;
        otherwise
            error('calcMetric: NotImplemented');
    end
    
end