% Simulate the continuous exploration of robotic hand with each finger
% being independently controlled.
% Save configurations as frames in an array and playback as a movie.

close all;
clc;

hand_file = load('allegro_hand_model.mat');
hand = hand_file.hand;
nFingers = hand.n;

h = figure(1);
hold on;
% h.Visible = 'off';

% M = []; % save as movie frame

iM = trajectoryVisualization(hand, hand.q, zeros(size(hand.q)), 10); % Open hand
% M = cat(1,M,iM);

hand = mySGmoveHand(hand, zeros(size(hand.q)));
oldHand = hand;

tgt_normal = zeros(nFingers, 3);
tgt_normal(:,3) = -1; % pointing towards -Z direction
tgt_normal = normalize(tgt_normal, 2, 'norm'); % normalize each row to unit vector

% tgt_position = zeros(nFingers, 3);
tgt_position = transpose(hand.ftips); % current fingertips positions are hand.ftips, (3,4)
tgt_position(:,3) = -50; % Keep Z at a same plane

fingerIndex = ones(hand.n); % index of fingers to explore

nSteps = 10; % exploration trials
nWaypoints = 10;

for i = 1:nSteps
    oldHand = hand; % continuous update
    
    if mod(i,2) % even step: explore in X direction, generate a random number
        offsetX = randi([-50,50],4,1);
        tgt_position(:,1) = tgt_position(:,1) + offsetX; % [TODO] The target position should lie inside the rmap
    else % odd step: explore in Y direction, generate a random number
        offsetY = randi([-50,50],4,1);
        tgt_position(:,2) = tgt_position(:,2) + offsetY;
    end
    
    hand = solveHandIK(hand, fingerIndex, tgt_position, tgt_normal);
    
    h = figure(1);
    hold on;
    % h.Visible = 'off';
    iM = trajectoryVisualization(hand, oldHand.q, hand.q, nWaypoints);
    % M = cat(1,M,iM);
    
end

disp('Exploration is over.');
% movie(M);