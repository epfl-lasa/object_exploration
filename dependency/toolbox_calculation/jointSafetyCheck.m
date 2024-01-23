function flag = jointSafetyCheck(q, lb, ub)
% Check if all joints are within lower bound and upper bound of its range
% Input: 
%     * q: joint angles to check
%     * lb: lower bound of joints
%     * ub: upper bound of joints
% Output:
%     * flag: true for inside the range
    
    flag = true;
    for i = 1:length(q)
        if isAlways(q(i)>ub(i)) || isAlways(q(i)<lb(i))
            flag = false;
            warning('Joint Safety Violated: joint %d, %.2f < %.2f < %.2f \n', i, lb(i),q(i),ub(i));
            return;
        end
    end
    
end