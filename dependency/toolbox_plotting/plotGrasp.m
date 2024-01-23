function plotGrasp(hand, obj, active, hold_on)
% Use the built-in functions of SG plot hand and object to plot both in a
% planned Grasp
if nargin < 4
    hold_on = false;
end

if strcmp(active, 'full')
    mySGplotHand(hand);
else
    mySGplotHand(hand, active);
end
hold on;
SGplotSolid(obj);

if hold_on
    hold on;
else
    hold off;
end

end