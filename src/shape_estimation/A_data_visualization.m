clc;
clear;
close all;

%% load data
file_dir = "../../database";
% file_name = "pointCloudTeapot"+".mat";
file_name = "pointCloudBottleNeck"+".mat";
% file_name = "pointCloudCylinder"+".mat";
% file_name = "pointCloudEllipseCylinder"+".mat";


data_ = load(file_dir + '/' + file_name);
data =  data_.ptCloud;
X_all = data.Location';
Xn_all = -data.Normal';

% point cloud
figure;
scatter3(X_all(1,:),X_all(2,:),X_all(3,:),5,'filled');
title("point cloud points");
xlabel("x");ylabel("y");zlabel("z");
hold on
quiver3(X_all(1,:),X_all(2,:),X_all(3,:),Xn_all(1,:),Xn_all(2,:),Xn_all(3,:),5.,'m','AutoScale','off');
axis('equal');

% figure
% scatter3(Xn(1,:),Xn(2,:),Xn(3,:),5,'filled');
% title("manifold of the normal form");

%% random sampling 
sample_no = 6000;
[X,ind_s]= datasample(X_all ,sample_no,2);
Xn = Xn_all(:,ind_s);

figure;
scatter3(X(1,:),X(2,:),X(3,:),5,'filled');
title("sampled point cloud data");
xlabel("x");ylabel("y");zlabel("z");
hold on
quiver3(X(1,:),X(2,:),X(3,:),Xn(1,:),Xn(2,:),Xn(3,:),1.,'m','AutoScale','off');
axis('equal');

%% data for surface

Y0 = zeros(1,length(X));
sc = 1.;
Yout = sc * rand(1,length(X));
Yin = -sc * rand(1,length(X));

Xout = X + repmat(Yout,size(Xn,1),1).*Xn;
Xin = X + repmat(Yin,size(Xn,1),1).*Xn;

figure;
scatter3(X(1,:),X(2,:),X(3,:),5,'filled');
hold on;
scatter3(Xin(1,:),Xin(2,:),Xin(3,:),5,'filled');
hold on;
scatter3(Xout(1,:),Xout(2,:),Xout(3,:),5,'filled');
title("point cloud for learning");
xlabel("x");ylabel("y");zlabel("z");
axis('equal');

%% save data for gpy

% toGpy.X = X;
% toGpy.V = Xn;
% toGpy.y = Y0;
% toGpy.Xin = Xin;
% toGpy.yin = Yin;
% toGpy.Xout = Xout;
% toGpy.yout = Yout;

save('toGpy_BottleNeck.mat','X','Xn','Xin','Yin','Xout','Yout');
% save('toGpy_EllipseCylinder.mat','X','Xn','Xin','Yin','Xout','Yout');

