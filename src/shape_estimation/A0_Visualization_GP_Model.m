clc;
clear;
close all;

%% load data
% file_dir = "../../database";
file_name = "test_of_gpy"+".csv";

data = readmatrix(file_name);
disp(size(data));

X = data(:,1:3)';
Y = data(:,4)';
pstd = data(:,end)';
figure
subplot(1,2,1);
pcshow(X',Y','MarkerSize',20)
set(gca,'color','w');
title('Depth Prediction Error at object surface','color','k');
colorbar;
subplot(1,2,2);
pcshow(X',pstd','MarkerSize',20)
set(gca,'color','w');
title('Prediciton std at object surface','color','k');
colorbar;
set(gcf,'color','w');

