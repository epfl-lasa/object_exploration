% Check the size of parameters before constructing the robot model. Function
% is called in 'mySGmakeFinger'.

function checkParams(DHpars,base,q)
    [rdh,cdh] = size(DHpars);
    [rb,cb] = size(base);
    [rq,cq] = size(q);

    if(cq ~= 1)
        error 'q is not a column vector'
    end
    
    if(rdh ~= rq)
        error 'DHpars number of rows must be the same as q number of columns'
    end
    
    if(rb ~= 4 || cb ~= 4 || sum(abs(base(4,1:3))) ~= 0 || base(4,4) ~= 1)
        error 'base is not a valid homogeneous transformation matrix'
    end
    
    if(cdh ~= 4)
       error 'Denavit-Hartenberg parameter matrix must have 4 columns' 
    end
end