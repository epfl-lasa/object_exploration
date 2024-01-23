% This script reads original data from ply files, process, and save to a
% local .mat file.

% listObjects = {'bunny','apc_red_bowl','apc_1','apc_2','apc_redcup'};

function pointCloud = plyFiles2PointCloud(fileName)
    if nargin < 1
        fileName = 'bunny';
    end

    % fileName: string, no file type
    p = pcread(strcat(fileName,'.ply'));
    
    % down sample
    gridStep = 0.002;
    pd = pcdownsample(p,'gridAverage',gridStep);
    pointCloud = pd.Location; % (N,3)
    
    try
        pointCloudNormals = pd.Normal;
    catch
        disp('Object normal does not exist, use approximation.');
        pointCloudNormals = pcnormals(pd);
    end
    % plot point cloud
%     pcshow(pd);

    %% Correct rotation
    if strcmp(fileName, 'bunny')
        pointCloud = transpose(rotx(90)*transpose(pointCloud));
    end

    %% Correct offset
    %%% Z: make bottom of object on the Z = 0 plane
    xrange = max(p.XLimits)-min(p.XLimits); % in meter
    yrange = max(p.YLimits)-min(p.YLimits);
    zrange = max(p.ZLimits)-min(p.ZLimits);

    offset = [mean(p.XLimits), mean(p.YLimits), min(p.ZLimits)];
    pointCloud = pointCloud - offset;

    N = size(pointCloud,1);

    fprintf('Object name: %s, size (m): [%d,%d,%d], number of samples: %d\n', fileName, xrange, yrange, zrange, N);

    figure, hold on;
    subplot(1,2,1); % show original point cloud
    pcshow(p);
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Original point cloud');

    subplot(1,2,2);
    scatter3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3));
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Corrected point cloud');
    hold off;
    
    save(strcat(fileName,'.mat'),'pd','pointCloud','pointCloudNormals');
end