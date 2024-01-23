function f = optExplore_objfun(X, X0, VecT)
    % X0: (1,43), X values from last step, X_init
    % VecT: (3,4), [ vt1x, vt2x, vt3x, vt4x; vt1y, vt2y, vt3y, vt4y; vt1z, vt2z, vt3z, vt4z]
    f = objfun(X, X0, VecT); % Vars: X_key, X0_sym, VecT
    % assert(isequal(class(f),'double'));
end