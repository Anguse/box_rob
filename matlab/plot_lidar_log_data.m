clear all; close all;

% A = load('/home/harald/ws/scan_match/logs/pos_controller/pos_log.txt');
A = load('logs/lidar_positions.txt');
% A = load('../logs/lidar_inliers.txt');


ind = 50000/100;
offset = 0;
% quality = A(:,1);
% angle = A(:,2)*pi/180+pi/2;
% distance = A(:,3);10

% X = cos(angle).*distance;  
% Y = sin(angle).*distance;

X = A(:,1)
Y = A(:,2)

n = numel(X);

fig = figure;
hax = axes;

hold on;
line([offset offset], [0 3635],get(hax,'YLim'),'Color',[1 0 0])
line([offset 2430+offset], [3635+offset 3635+offset],get(hax,'YLim'),'Color',[1 1 0])
line([2430+offset 2430+offset], [3635+offset offset],get(hax,'YLim'),'Color',[1 0 0])
line([2430+offset offset], [offset offset], get(hax,'YLim'),'Color',[1 0 0])
for i = 1:5:n
    X(i) = X(i);
    Y(i) = Y(i);
    plot(X(i),Y(i),'*');
    pause(0.00005)
end


