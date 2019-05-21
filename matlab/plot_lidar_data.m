close all; clear all;

% A = load('lidar_measurements.txt');
% B = load('lidar_positions.txt');
% C = load('lidar_log.txt');
% D = load('lidar_inliers.txt');

%angle = A(:,2);
%dist = A(:,3);
offset = 0;
fig = figure;
hax = axes;
%X = cos(angle).*dist;
%Y = sin(angle).*dist;
X = B(1:10:end,1);
Y = B(1:10:end,2);

hold on;
line([offset offset], [0 3635],get(hax,'YLim'),'Color',[1 0 0])
line([offset 2430+offset], [3635 3640],get(hax,'YLim'),'Color',[1 0 0])
line([2430+offset 2430+offset], [3640 0],get(hax,'YLim'),'Color',[1 0 0])
line([2430+offset offset], [0 0],get(hax,'YLim'),'Color',[1 0 0])
n = numel(X);
for i = 1:n
    %if ((X(i)<4000&&X(i)>-2) && (Y(i)<2500&&Y(i)>-2))
    plot(X(i), Y(i), '-.g*');
    pause(0.0001)
end

% X = C(:,1);
% Y = C(:,2);
% n = numel(X);
% for i = 1:n
%     %if ((X(i)<4000&&X(i)>-2) && (Y(i)<2500&&Y(i)>-2))
%     plot(X(i), Y(i),'-.r*');
%     pause(0.0001)
%     %end
% end
% X = B(:,1);
% Y = B(:,2);
% n = numel(X);
% for i = 1:n
%     %if ((X(i)<4000&&X(i)>-2) && (Y(i)<2500&&Y(i)>-2))
%     plot(X(i), Y(i),'-.b*');
%     pause(0.0001)
%     %end
% end
% X = A(:,1);
% Y = A(:,2);
% n = numel(X);
% for i = 1:n
%     %if ((X(i)<4000&&X(i)>-2) && (Y(i)<2500&&Y(i)>-2))
%     plot(X(i), Y(i),'-.b*');
%     pause(0.0001)
%     %end
% end