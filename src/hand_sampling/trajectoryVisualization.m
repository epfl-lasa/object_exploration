% This script visualizes the trajectory of robotic hand [deprecated]

function [M] = trajectoryVisualization(hand, q_init, q_tgt, timeSteps)
    % Setup
    % clc, close all;

    if nargin < 2
        q_init = hand.limit(:,1);
    end
    if nargin < 3
        q_tgt = hand.limit(:,2);
    end
    if nargin < 4
        timeSteps = 10;
    end

    % figure, hold on;

    if nargout > 0
        loops = timeSteps;
        M(loops) = struct('cdata',[],'colormap',[]);
    end
    for i = 1:timeSteps
        clf;
        q = q_init + i*(q_tgt - q_init)/timeSteps;
        hand = mySGmoveHand(hand, q);
        mySGplotHand(hand);
        view([180,30]);
        axis off;
        % grid off;
        drawnow;
        if nargout > 0
            M(i) = getframe;
        end
    end
end