function [train_data, test_data] = sample_from_ptc(name_file,n_train,n_test,n_off,bool)

% file_loc = "/home/feri/git/object_shape_exploration/obj_file";
file_loc = 'C:\Users\break\Workspace\inhand_exploration\obj_file';

ptCloud = pcread(file_loc+"/"+name_file+".ply");
% pcshow(ptCloud);
gridStep = 0.002;
ptCloudA = pcdownsample(ptCloud,'gridAverage',gridStep);
normals = pcnormals(ptCloudA);

Xdata = double(ptCloudA.Location);
fprintf('downsmapled point cloud has %d points.\n',ptCloudA.Count);
Xmax = max(Xdata);
Xmin = min(Xdata);

scale = 0.25;
% center and normalize
if(bool == true)
    b=-0.5*(Xmax + Xmin);
    a=max(0.5*(Xmax - Xmin));
    A = (scale/a)*eye(3);
    Xcent = (Xdata +b)*A;
    Xdata = Xcent;
end
%
indx = randperm(size(Xdata,1));
Xout =[];
Xin = [];

% lb = Xmin - 0.1*(Xmax - Xmin);
% ub = Xmax + 0.1*(Xmax - Xmin);
% x = linspace(lb(2),ub(2),50);
% y = linspace(lb(2),ub(2),50);
% z = linspace(lb(3),ub(3),50);
% 
% [ii,jj] = meshgrid(1:50,1:50);
% ig = ii(:);jg = jj(:);
% for i=1:length(ig)
%     pt = zeros(6,3);
%     pt(1,:) = [lb(1) y(ig(i)) z(jg(i))];
%     pt(2,:) = [ub(1) y(ig(i)) z(jg(i))];
%     pt(3,:) = [x(ig(i)) lb(2) z(jg(i))];
%     pt(4,:) = [x(ig(i)) ub(2) z(jg(i))];
%     pt(5,:) = [x(ig(i)) y(jg(i)) lb(3)];
%     pt(6,:) = [x(ig(i)) y(jg(i)) ub(3)];
% 
%     for j=1:6
%         [ind, ~] =  findNearestNeighbors(ptCloud,pt(j,:),20);
%         ind = datasample(ind,1);
%         vn = normals(ind,:);
%         xn = double(ptCloud.Location(ind,:));
% 
%         ed = normalize(pt(j,:)-xn);
%         if(ed*vn' < 0)
%             vn = -vn;
%         end
%         dis = 0.01;
%         Xout = [Xout; xn+dis*vn];
%         Xin = [Xin; xn-dis*vn];
%     end
% 
% end
[xx,yy,zz] = sphere(ceil(sqrt(10*n_off+1)));
a = max(1.5*(Xmax - Xmin));
xyz = a*[xx(:) yy(:) zz(:)];
for i = 1:length(xyz)
   pt = xyz(i,:);
   [ind, ~] =  findNearestNeighbors(ptCloudA,pt,20);
    ind = datasample(ind,1);
    vn = normals(ind,:);
    xn = Xdata(ind,:);

    ed = (pt-xn)/norm(pt-xn);
    if(ed*vn' < 0)
        vn = -vn;
    end
    dis = 0.015+0.01*rand;
    Xout = [Xout; xn+dis*vn];
    Xin = [Xin; xn-dis*vn];
    
end

indy = randperm(size(Xout,1));
X_out = Xout(indy(1:2*n_off),:);
X_in = Xin(indy(1:2*n_off),:);
   
%% training and test sets:
train_data.X = Xdata(indx(1:n_train),:); 
train_data.Xout = X_out(1:n_off,:);
train_data.Xin = X_in(1:n_off,:);
train_data.Y = zeros(size(train_data.X,1),1);
train_data.Yout = 1*ones(size(train_data.Xout,1),1);
train_data.Yin = -1*ones(size(train_data.Xin,1),1);


test_data.X = Xdata(indx(1+n_train:n_train+n_test),:);
test_data.Xout = X_out(1+n_off:end,:);
test_data.Xin = X_in(1+n_off:end,:);
test_data.Y = zeros(size(test_data.X,1),1);
test_data.Yout = 1*ones(size(test_data.Xout,1),1);
test_data.Yin = -1*ones(size(test_data.Xin,1),1);

end