d = load('pointCloudTeapot.mat');
d = d.ptCloud;

x = d.Location(:,1);
y = d.Location(:,2);
z = d.Location(:,3);

u = d.Normal(:,1); % may not exist for all objects
v = d.Normal(:,2);
w = d.Normal(:,3);

% Force all normal vectors pointing inwards
center = [0,0,(max(z)+min(z))/2];

for k = 1:numel(x)
    p1 = center - [x(k),y(k),z(k)];
    p2 = [u(k),v(k),w(k)];
    angle = atan2(norm(cross(p1,p2)),p1*p2');
    if angle > pi/2 || angle < -pi/2
        u(k) = -u(k);
        v(k) = -v(k);
        w(k) = -w(k);
    end
end

% Save updated data as mat
ptCloud = d;
ptCloud.Normal = [u,v,w];
save('pointCloudTeapot.mat', 'ptCloud');

% Save updated data as csv
pointCloudTeapot = cat(2, ptCloud.Location, ptCloud.Normal);
csvwrite('pointCloudTeapot.csv', pointCloudTeapot);

% a = readtable('pointCloudTeapot.csv'); % check csv data