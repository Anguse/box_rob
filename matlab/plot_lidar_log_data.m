clear all; close all;

A = load('lidar_measurements.txt');
B = load('lidar_adjustment.txt');

n = numel(X);
ind = 50000/100;
offset = 0;

angle = A(50000:end,2);
dist = A(50000:end,3);

X = cos(angle).*dist;  
Y = sin(angle).*dist;

fig = figure;
hax = axes;

hold on;
line([offset offset], [0 3635],get(hax,'YLim'),'Color',[1 0 0])
line([offset 2421+offset], [3635 3640],get(hax,'YLim'),'Color',[1 0 0])
line([2421+offset 2421+offset], [3640 0],get(hax,'YLim'),'Color',[1 0 0])
line([2430+offset offset], [0 0],get(hax,'YLim'),'Color',[1 q0 0])
for i = 1:n
    if mod(i,100) == 0
        ind = ind+1;
    end
    X(i) = X(i) + B(ind,1);
    Y(i) = Y(i) + B(ind,2);
    plot(Y(i),X(i),'*');
    pause(0.00005)
end


