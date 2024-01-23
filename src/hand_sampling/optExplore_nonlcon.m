function [c, ceq] = optExplore_nonlcon(X, next_contacts, baseMatrices, objectValues, quatH_old)
    
    ceq = nonl_ceq(X, next_contacts.positions, next_contacts.normals);
    
    c = nonl_c(X, baseMatrices.fingers, baseMatrices.palm, objectValues.center, objectValues.dist.min, quatH_old);

end