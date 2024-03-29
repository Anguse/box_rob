close all; clear all;

 A = load('/home/harald/ws/scan_match/logs/pos_controller/odometry.txt');

offset = 0;
fig = figure;
hax = axes;

X = A(:,1);
Y = A(:,2);

hold on;
line([offset offset], [0 3635],get(hax,'YLim'),'Color',[1 0 0])
line([offset 2430+offset], [3635 3640],get(hax,'YLim'),'Color',[1 0 0])
line([2430+offset 2430+offset], [3640 0],get(hax,'YLim'),'Color',[1 0 0])
line([2430+offset offset], [0 0],get(hax,'YLim'),'Color',[1 0 0])
n = numel(X);
for i = 1:10:n
    plot(X(i), Y(i), '-.g*');
    pause(0.0001)
end