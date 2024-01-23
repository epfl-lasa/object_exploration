function plotGraspTrial(i, hand, obj)
% plot the intermediate results of grasping search trial
    figure(i);
    subplot(1,2,1);
    grid on;
    SGplotHand(hand);
    hold on;
    SGplotSolid(obj);
    
    figure(i);
    subplot(1,2,2);
    grid on;
    SGplotHand(hand_c);
    hold on;
    SGplotSolid(obj);
end