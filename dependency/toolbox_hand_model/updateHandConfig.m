function hand = updateHandConfig(hand, soln)
% Update hand configurations given chosen optimization result
% 1. Move finger according to the joint positions given in X
% 2. Set all affected joints (before contact point's link) to deactive
% 
% soln: a struct that contains all solution information
%   - X_sol: solution variables
%   - param: parameter configurations corresponds to the optimal solution

X_sol = soln.X_sol;
param = soln.param;

ncp = param.ncp;
os = param.os;
os_info = os.os_info;

X_key = param.X_key;
dict_q = param.dict_q;
idx_q = param.idx_q;

qactv = param.qactv_loop; % save active joints used in this loop

% Step 1: activate the joints used for current contacts
for i = 1:ncp
    hand = activateLinkContact(hand, os_info{i}(1), os_info{i}(2));
end

% Step 2: move hand joints
q = hand.q;
if ~isequal(X_key(idx_q), dict_q(qactv))
    warning('Incorrect');
    disp(X_key);
    disp(idx_q);
    disp(X_key(idx_q));
    disp(qactv);
    disp(dict_q(qactv));
else
    q(qactv) = X_sol(idx_q); % update current hand joints with the solution states
end

hand = mySGmoveHand(hand,q);

% Step 3: disablt the joints that are in contact
% Notice that the joint active states of hand are used through the entire code

for i = 1:ncp
    hand = deactivateLinkContact(hand, os_info{i}(1), os_info{i}(2));
end

% disp('Nach dem Update'); disp(hand.qactv);

% Step 4: substitude values of the deactivated joints in symbolic fields,
% such as contact points 'Cp' and contact normal 'Cn', using the
% corresponding values in X_sol. Indexing via X_key.
disp('  Updating symbolic expressions...');

%%% Notice that any symbolic expression that has not been used in
%%% calculating the constrains / objective function can be ignored and
%%% remain unupdated to save running time.

hand.symbolic.Cp = subs(hand.symbolic.Cp, X_key(idx_q), X_sol(idx_q));
hand.symbolic.Cn = subs(hand.symbolic.Cn, X_key(idx_q), X_sol(idx_q));
hand.symbolic.Cr = subs(hand.symbolic.Cr, X_key(idx_q), X_sol(idx_q));

for i = 1:hand.n % iterate over fingers

    % Finger Symbolic
    hand.F{i}.symbolic.base       = subs(hand.F{i}.symbolic.base,       X_key(idx_q), X_sol(idx_q));
    hand.F{i}.symbolic.joints_pos = subs(hand.F{i}.symbolic.joints_pos, X_key(idx_q), X_sol(idx_q));
    hand.F{i}.symbolic.tip        = subs(hand.F{i}.symbolic.tip,        X_key(idx_q), X_sol(idx_q));
    
    for j = 1:hand.F{i}.nlink % iterate over finger links

        % Link Symbolic
        hand.F{i}.Link{j}.symbolic.HT_this   = subs(hand.F{i}.Link{j}.symbolic.HT_this,   X_key(idx_q), X_sol(idx_q));
        hand.F{i}.Link{j}.symbolic.HT_next   = subs(hand.F{i}.Link{j}.symbolic.HT_next,   X_key(idx_q), X_sol(idx_q));
        hand.F{i}.Link{j}.symbolic.link_dist = subs(hand.F{i}.Link{j}.symbolic.link_dist, X_key(idx_q), X_sol(idx_q));
        
        hand.F{i}.Link{j}.contact.symbolic.n = subs(hand.F{i}.Link{j}.contact.symbolic.n, X_key(idx_q), X_sol(idx_q));
        hand.F{i}.Link{j}.contact.symbolic.p = subs(hand.F{i}.Link{j}.contact.symbolic.p, X_key(idx_q), X_sol(idx_q));
        
        hand.F{i}.Link{j}.contact.symbolic.lc      = subs(hand.F{i}.Link{j}.contact.symbolic.lc,      X_key(idx_q), X_sol(idx_q));
        hand.F{i}.Link{j}.contact.symbolic.cp_dist = subs(hand.F{i}.Link{j}.contact.symbolic.cp_dist, X_key(idx_q), X_sol(idx_q));
        
        hand.F{i}.Link{j}.contact.symbolic.FC =          subs(hand.F{i}.Link{j}.contact.symbolic.FC,          X_key(idx_q), X_sol(idx_q));
        hand.F{i}.Link{j}.contact.symbolic.TC =          subs(hand.F{i}.Link{j}.contact.symbolic.TC,          X_key(idx_q), X_sol(idx_q));
    end

end

disp('  Symbolic expressions updated.');

end
