function ptCloud = create_point_cloud(type, T, scaling)

    close all;
    
    if nargin < 3
        scaling = 10;
    end
    if nargin < 2
        T = eye(4);
%         T(1:3,1:3) = rotx(60);
        T(1:3,4) = [0,0,-200];
    end
    if nargin < 1
        type = 'cylinder';
%         type = 'ellipse_cylinder';
        % type = 'bottle_neck';
    end
    
    switch type
        case 'cylinder'
            ptCloud = createCylinder(T,scaling);
        case 'ellipse_cylinder'
            ptCloud = createEllipseCylinder(T,scaling);
        case 'bottle_neck'
            ptCloud = createBottleNeck(T,scaling);
    end
    
end



function ptCloud = createCylinder(T,scaling)
    % Create a cylinder
    R = 5;
    N = 100; % number of points on the same plane
    H = 20; % height of the cylinder
    HN = 50; % number of points along the Z-axis
    
    [xc,yc,~] = cylinder(R,N); % xc,yc,zc:(2,N+1)
    
    Z = repmat(linspace(1,H,HN).', 1, N+1); % (HN, N+1)
    X = repmat(xc(1,:), HN, 1);
    Y = repmat(yc(1,:), HN, 1);
    
    x = X(:).*scaling; % (HN*(N+1), 1)
    y = Y(:).*scaling;
    z = Z(:).*scaling;
    
    if ~isequal(T, eye(4))
        data_h = [x';y';z';ones(1,HN*(N+1),1)]; % (4, HN*(N+1))
        data_h = T*data_h;
        x = data_h(1,:)';
        y = data_h(2,:)';
        z = data_h(3,:)';
    end
    
    %%% Add noise to the data points
    sigma = 0.05;
    x = normrnd(x,sigma);
    y = normrnd(y,sigma);
    z = normrnd(z,sigma);
    
    p = pointCloud([x,y,z]);
    normals = pcnormals(p);
    
    u = normals(:,1);
    v = normals(:,2);
    w = normals(:,3);
    
    %%% Correct normal directions
    pcCenter = [0,0];
    for k = 1:numel(x)
        p1 = [pcCenter(1)-x(k),pcCenter(2)-y(k),0];
        p2 = [u(k),v(k),w(k)];
        angle = atan2(norm(cross(p1,p2)),p1*p2');
        if angle > pi/2 || angle < -pi/2
            u(k) = -u(k);
            v(k) = -v(k);
            w(k) = -w(k);
        end
    end

    quiver3(x,y,z,u,v,w);
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
    ptCloud = p;
    ptCloud.Normal = [u,v,w];
    save('pointCloudCylinder.mat', 'ptCloud');

    pointCloudCylinder = cat(2, ptCloud.Location, ptCloud.Normal);
    csvwrite('pointCloudCylinder.csv', pointCloudCylinder);
end



function ptCloud = createEllipseCylinder(T,scaling)
    % Create a cylinder
    a = 5; % minor axis
    b = 10; % major axis
    
    N = 100; % number of points on the same plane
    H = 20; % height of the cylinder
    HN = 50; % number of points along the Z-axis
    
    t = linspace(-pi,pi,N+1);
    t = t(1:end-1);
    xc = 0 + a*cos(t);
    yc = 0 + b*sin(t);
    
    Z = repmat(linspace(1,H,HN).', 1, N); % (HN, N+1)
    X = repmat(xc(1,:), HN, 1);
    Y = repmat(yc(1,:), HN, 1);

    x = X(:).*scaling;
    y = Y(:).*scaling;
    z = Z(:).*scaling;
    
    if ~isequal(T, eye(4))
        data_h = [x';y';z';ones(1,HN*(N+1),1)]; % (4, HN*(N+1))
        data_h = T*data_h;
        x = data_h(1,:)';
        y = data_h(2,:)';
        z = data_h(3,:)';
    end
    
    %%% Add noise to the data points
    sigma = 0.05;
    x = normrnd(x,sigma);
    y = normrnd(y,sigma);
    z = normrnd(z,sigma);
    
    p = pointCloud([x,y,z]);
    normals = pcnormals(p);
    
    u = normals(:,1);
    v = normals(:,2);
    w = normals(:,3);
    
    %%% Correct normal directions
    pcCenter = [0,0];
    for k = 1:numel(x)
        p1 = [pcCenter(1)-x(k),pcCenter(2)-y(k),0];
        p2 = [u(k),v(k),w(k)];
        angle = atan2(norm(cross(p1,p2)),p1*p2');
        if angle > pi/2 || angle < -pi/2
            u(k) = -u(k);
            v(k) = -v(k);
            w(k) = -w(k);
        end
    end
    
    quiver3(x,y,z,u,v,w);
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
    ptCloud = p;
    ptCloud.Normal = [u,v,w];
    save('pointCloudEllipseCylinder.mat', 'ptCloud');

    pointCloudEllipseCylinder = cat(2, ptCloud.Location, ptCloud.Normal);
    csvwrite('pointCloudEllipseCylinder.csv', pointCloudEllipseCylinder);
end


function ptCloud = createBottleNeck(T,scaling)

    N = 100; % number of points on the same plane
    H = 20; % height of the cylinder
    
    t = linspace(0,2*pi,N);
    r = 2 + cos(t);
    [xc,yc,zc] = cylinder(r,N);
    
    X = xc(:);
    Y = yc(:);
    Z = zc*H;

    x = X(:).*scaling;
    y = Y(:).*scaling;
    z = Z(:).*scaling;
    
    if ~isequal(T, eye(4))
        data_h = [x';y';z';ones(1,HN*(N+1),1)]; % (4, HN*(N+1))
        data_h = T*data_h;
        x = data_h(1,:)';
        y = data_h(2,:)';
        z = data_h(3,:)';
    end
    
    %%% Add noise to the data points
    sigma = 0.05;
    x = normrnd(x,sigma);
    y = normrnd(y,sigma);
    z = normrnd(z,sigma);
    
    p = pointCloud([x,y,z]);
    normals = pcnormals(p);
    
    u = normals(:,1);
    v = normals(:,2);
    w = normals(:,3);
    
    %%% Correct normal directions
    pcCenter = [0,0];
    for k = 1:numel(x)
        p1 = [pcCenter(1)-x(k),pcCenter(2)-y(k),0]; % lie inside the same plane
        p2 = [u(k),v(k),w(k)];
        angle = atan2(norm(cross(p1,p2)),p1*p2');
        if angle > pi/2 || angle < -pi/2
            u(k) = -u(k);
            v(k) = -v(k);
            w(k) = -w(k);
        end
    end
    
    %%% Add noise here if expect smooth surface normal
    
    quiver3(x,y,z,u,v,w);
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
    ptCloud = p;
    ptCloud.Normal = [u,v,w];
    save('pointCloudBottleNeck.mat', 'ptCloud');

    pointCloudBottleNeck = cat(2, ptCloud.Location, ptCloud.Normal);
    csvwrite('pointCloudBottleNeck.csv', pointCloudBottleNeck);

end