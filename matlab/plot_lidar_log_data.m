clear all; close all;

% A = load('lidar_measurements.txt');
% B = load('lidar_adjustment.txt');
A = load('../logs/lidar_inliers.txt');


ind = 50000/100;
offset = 0;

X = A(:,1);
Y = A(:,2);

% X = cos(angle).*dist;  
% Y = sin(angle).*dist;

n = numel(X);

fig = figure;
hax = axes;

hold on;
line([offset offset], [0 3635],get(hax,'YLim'),'Color',[1 0 0])
line([offset 2421+offset], [3635 3640],get(hax,'YLim'),'Color',[1 0 0])
line([2421+offset 2421+offset], [3640 0],get(hax,'YLim'),'Color',[1 0 0])
line([2430+offset offset], [0 0],get(hax,'YLim'),'Color',[1 0 0])
for i = 1:n
    X(i) = X(i);
    Y(i) = Y(i);
    plot(Y(i),X(i),'*');
    pause(0.00005)
end


