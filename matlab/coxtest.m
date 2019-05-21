%% Example Title
% Co-ordinates of the ref. nodes
close all; clear all;
REF = [0 0;   % 01
       0 3635;  % 02
       2430 3635;  % 03
       2430 0;]% 04

% LINE SEGMENT MODEL (Indeces in the REF-vector)
LINES = [1 2;       % L01
         2 3;       % L02
         3 4;       % L03
         4 1;]       % L04
         
 LINEMODEL = [REF(LINES(:,1),1:2) REF(LINES(:,2),1:2)];

 A = load('lidar_recording.txt');
 angs = A(1:50000,2);     % Angles in radians
 dist = A(1:50000,3);     % Distances
 
 posX = 1215;
 posY = 160;
 posA = 0;
 
 alfa = 0;
 beta = 0;
 gamma = 0;
 
 coX(1) = posX;
 coY(1) = posY;
 coA(1) = posA;
 
 [no_inputs u] = size(angs)
 
 set_ptr = 1;
 set_size = 99;
 
 C(1,1:9) = [1, 0, 0, 0, 1, 0, 0, 0, (pi/180)^2];
 dC = C;
 
 %% Loop
 for i = 2:no_inputs/set_size,
    
     angle_set = angs(set_ptr:set_ptr+set_size);
     dist_set = dist(set_ptr:set_ptr+set_size);
     
     [dx, dy, da, dC] = Cox_LineFit_h(angle_set, dist_set, [coX(i-1) coY(i-1) coA(i-1)]', [alfa beta gamma]', LINEMODEL);
     
     coX(i-1) = coX(i-1) + dx;
     coY(i-1) = coY(i-1) + dy;
     coA(i-1) = coA(i-1) + da;
     
     set_ptr = set_ptr + set_size;
     
     coX(i) = coX(i-1);
     coY(i) = coY(i-1);
     coA(i) = coA(i-1);
     
     C(i-1,1:3) = dC(1,1:3);
     C(i-1,4:6) = dC(2,1:3);
     C(i-1,7:9) = dC(3,1:3);
     i
 end

 %% Plot
 fig = figure;
 hax = axes;
 hold on;
 
line([0 0], [0 3635],get(hax,'YLim'),'Color',[1 0 0])
line([0 2430+0], [3635 3640],get(hax,'YLim'),'Color',[1 0 0])
line([2430+0 2430+0], [3640 0],get(hax,'YLim'),'Color',[1 0 0])
line([2430+0 0], [0 0],get(hax,'YLim'),'Color',[1 0 0])

[no_adjustments u] = size(coX);

for i = 1:no_adjustments,
    plot(coX(i), coY(i),'-.r*');
    pause(0.0001)
end
