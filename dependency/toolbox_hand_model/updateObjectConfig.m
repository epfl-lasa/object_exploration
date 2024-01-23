function object = updateObjectConfig(object, soln)

X_sol = soln.X_sol;
param = soln.param;

switch object.type
    case 'sph' % sphere
        oc = X_sol(param.idx_oc); % object center
        To = trvec2tform(oc(:).');
        radius = param.object_radius; % object radius
        
        object = mySGsphere(To,radius,object.clr);
    otherwise
    	error('NotImplementedError.');
end