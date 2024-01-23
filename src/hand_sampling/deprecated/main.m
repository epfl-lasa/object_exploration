%%% This script analyzes the hand-object relationship in in-hand
%%% exploration task.

% cd(hand_sampling_dir); % go to current directory
database_dir = '..\..\database\'; % Execute code at main package

%%% Construct hand model
if exist(fullfile(database_dir, 'allegro_hand_model.mat'),'file')
    data = load(fullfile(database_dir, 'allegro_hand_model.mat'));
    hand = data.hand;
else
    Th = eye(4);
    Th(1:3,4) = [0;0;0];
    Th(1:3,1:3) = eye(3);
    hand = mySGallegroLeft(Th);
    save(fullfile(database_dir, 'allegro_hand_model.mat'),'hand');
end
disp('Hand model loaded.');

%%% Construct reachability map
if exist(fullfile(database_dir, 'allegro_hand_rmap_model.mat'),'file')
    data = load(fullfile(database_dir, 'allegro_hand_rmap_model.mat'));
    hand_map = data.rmap;
else
    if_plot = true;
    if_save = false;
    [hand_map, hand] = reachabilityMap(hand, 'all', if_plot, if_save);
    save(fullfile(database_dir, 'allegro_hand_rmap_model.mat'),'rmap');
end
disp('Hand reachability map loaded.');

%%% Load object model
scaling = 10; % used to control ratio
T = eye(4); % homogeneous transformation matrix
T(1:3,4) = [0,0,-250];

object = create_point_cloud('cylinder', T, scaling);
disp('Object model loaded.');

%%% Visualization of hand and object
figure, hold on;
quiver3(object.Location(:,1),object.Location(:,2),object.Location(:,3),...
    object.Normal(:,1),object.Normal(:,2),object.Normal(:,3));
axis equal;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
mySGplotHand(hand);
hold off;

% list the finger links to plot their reachability maps
links2plot = {[4],... % first cell element corresponds to thumb, '4' indicates the 4th link (distal phalax)
    [4],... % second cell element corresponds to the 2nd finger, the index finger
    [4],... % the middle finger
    [4],... % the ring finger (the last one, Allegro hand has only 4 fingers, but human hand model has 5)
    [0]}; % last element corresponds to the palm. 1: plot palm, 0: do not plot palm

plotReachabilityMap(hand, hand_map, links2plot);

%%% Analyze hand-object relationship
% detect collide finger links and region
detect_contact_region(object, hand_map, links2plot);