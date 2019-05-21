%   Cox_LineFit(angs, meas, [X(kk-1) Y(kk-1) A(kk-1)]', LINEMODEL, cox_counter, REF, LINES)
%   OUTPUT [dx dy da C]        
%
function [dx, dy, da, C] = Cox_LineFit(angs, meas, posMatrix, LINEMODEL, cox_counter, REF, LINES)
    %fprintf('You are now in the Cox calculation for the %d time.\n', cox_counter);
    [no_rows, no_cols] = size(LINEMODEL);
    dx = 0; dy = 0; da = 0;
    C = [0 0 0; 0 0 0; 0 0 0];
    
    % Calculate unit vectors for all lines
    for kk = 1:no_rows
        R = [0 -1;
            1 0];
        V = [LINEMODEL(kk,1) LINEMODEL(kk,3);   % X1, Y1;
            LINEMODEL(kk,2) LINEMODEL(kk,4)];   % X2, Y2;
        T = [LINEMODEL(kk,1) LINEMODEL(kk,1);   % X1 X1;
            LINEMODEL(kk,2) LINEMODEL(kk,2)];   % Y1 Y1
        V2 = R*(V-T);
        U(kk,1:2) = V2(1:2,2')/sqrt(V2(1,2)^2+V2(2,2)^2);
        RI(kk,1) = dot(U(kk,1:2),LINEMODEL(kk,1:2));
        
        % Calculates the midpoint and the length of the line
        MIDPOINT = [(LINEMODEL(kk,1)+LINEMODEL(kk,3))/2, (LINEMODEL(kk,2)+LINEMODEL(kk,4))/2];
        LINELENGTH = sqrt((LINEMODEL(kk,1)-LINEMODEL(kk,3))^2+(LINEMODEL(kk,2)-LINEMODEL(kk,4))^2);
        
        % Matrix for the lines, midpoints and lengths with
        %  1  2  3  4  5  6      7
        %[x1 y1 x2 y2 xm ym length*0.5]
        LINEMATRIX(kk,1:7) = [LINEMODEL(kk,1:4) MIDPOINT(1:2) LINELENGTH*0.5];
    end
    
    % The position of the robot
    posX = posMatrix(1);
    posY = posMatrix(2);
    posA = posMatrix(3);
    
    %% Loop
    n = 1; % Loop counter
    while n <= 10
        %fprintf('This is loop number %d : %d\n', cox_counter, n);
        
        % Change the position
        posX = posX + dx;
        posY = posY + dy;
        posA =  mod(posA + da, 2*pi);
        
        % Counts the number of laser measurements
        number_of_laser_measurements = max(size(angs));
        
        % Loop to translate and rotate the data points
        % Finds the nearest line
        % Output is the matrix called Xw
        for nn = 1:number_of_laser_measurements
            % Sensor to cartesian
            measX = meas(nn)*cos(angs(nn));
            measY = meas(nn)*sin(angs(nn));
        
            % Position of the sensor on the robot
            alfa = 660; beta = 0; gamma = -90*pi/180;

            % Sensor coordinates to robot coordinates
            R = [cos(gamma) -sin(gamma) alfa;
                sin(gamma) cos(gamma) beta;
                0 0 1];
            Xs = R*[measX measY 1]';
            
            % Robot coordinates to world coordinates
            RC = [cos(posA) -sin(posA) posX;
                sin(posA) cos(posA) posY;
                0 0 1];
            
            % Put the coordinates into the Xw matrix with
            %     1       2       3   4   5
            % [number x-coord y-coord 1 10000]
            % Position 4 and 5 are dummy variables in this stage
            Xw(nn,1:5) = [nn (RC*[Xs(1) Xs(2) 1]')' 10000];
            
            % Loop to find nearest line
            % Update the Xw matrix with the nearest line and distance
            %     1       2       3                4                 5
            % [number x-coord y-coord number_of_the_nearest_line distance]
            for mm = 1:no_rows
                dist = RI(mm,1)- dot(U(mm,1:2), Xw(nn,2:3));
                if abs(dist) <= abs(Xw(nn,5))
                    Xw(nn,4) = mm;
                    Xw(nn,5) = dist;
                end
            end 
        end
        %disp(Xw)
        
        % To get the median distance value
        Xm = abs(Xw(:,5));
        M = median(Xm);
        
        % Counter for the new Vi matrix (a matrix with outliers removed)
        matrix_counter = 0;
        
        % Reject outliers is done by:
        % 1. Checking the distance from measurement to the line
        % 2. Checking that the distance from the laser point to the 
        % midpoint of the line is not more than 0.6*length of the line
        % This is done since the calculations conciders the lines as
        % infintely long
        % To create the Vi matrix with
        %        1          2       3       4       5       6
        % [nearest_line x-coord y-coord distance normalX normalY]
        for nn = 1:number_of_laser_measurements
            if (abs(Xw(nn,5))<=25)&&((sqrt((Xw(nn,2)-LINEMATRIX(Xw(nn,4),5))^2+((Xw(nn,3)-LINEMATRIX(Xw(nn,4),6))^2)))<=(LINEMATRIX(Xw(nn,4),7)))
                matrix_counter = matrix_counter + 1;
                Vi(matrix_counter,1:6) = [Xw(nn,4) Xw(nn,2) Xw(nn,3) Xw(nn,5) U(Xw(nn,4),1) U(Xw(nn,4),2)];
            end
        end
        
        % If the number of measurements are very small we change the distance limit
        if matrix_counter < 10
            for nn = 1:number_of_laser_measurements
                if (abs(Xw(nn,5))<=(M/2))&&((sqrt((Xw(nn,2)-LINEMATRIX(Xw(nn,4),5))^2+((Xw(nn,3)-LINEMATRIX(Xw(nn,4),6))^2)))<=(LINEMATRIX(Xw(nn,4),7)))
                    matrix_counter = matrix_counter + 1;
                    Vi(matrix_counter,1:6) = [Xw(nn,4) Xw(nn,2) Xw(nn,3) Xw(nn,5) U(Xw(nn,4),1) U(Xw(nn,4),2)];
                end
            end
        end
     
        %disp(Vi)
        ViCopy = Vi;

        % Set up the linear equation system
        % First the parts of the matrix A
        Xi1(:,1) = Vi(:,5)
        Xi2(:,1) = Vi(:,6);
        for rr = 1:matrix_counter
            Xi3(rr,1) = [Vi(rr,5) Vi(rr,6)]*[0 -1; 1 0]*([Vi(rr,2) Vi(rr,3)]' - [mean(Xw(:,2)) mean(Xw(:,3))]');
        end
        % To find B = [dx dy da]'
        Y = Vi(:,4)
        A = [Xi1 Xi2 Xi3];
        % To calculate B = inv(A'*A)*A'*Y
        B = A\Y;
        
        % Calculate the covariance matrix (C)
        S2 =(Y-A*B)'*(Y-A*B)/((max(size(A)))-4);
        C = S2*(inv(A'*A));

        % Add latest contribution to the change in position and angle
        dx = dx + B(1);
        dy = dy + B(2);
        da = da + B(3);
        
        % To display the differences calculated in this step
        %diff = B'
        %total_diff = [dx dy da]
        
        % Check if the loop should be stopped
        if (sqrt(B(1)^2 + B(2)^2) < 15)&&(abs(B(3) < 0.1*pi/180))
            n = 100;
        end       
        
        % Clearing variables to avoid error
        clear Xi1;
        clear Xi2;
        clear Xi3;
        clear A;
        Vi = [0 0 0 0 0 0];
        
        % Emergency stop
        if (abs(dx)>100)||(abs(dy)>100)||(abs(da)>0.5)
            dx = 0;
            dy = 0;
            da = 0;
            n = 100;
        end
        
        n = n + 1; % Update loop counter
    end
    
    %% End of the loop
    % Plot to see the result
    plot(ViCopy(:,2), ViCopy(:,3), 'b*');
    hold on;
    for qq = 1:max(size(LINES))
        X = [REF(LINES(qq,1), 1) REF(LINES(qq,2), 1)]';
        Y = [REF(LINES(qq,1), 2) REF(LINES(qq,2), 2)]';
        plot(X, Y, 'r');
        plot(X, Y, 'ro');
    end
    axis([0 12000 1000 10000]);
    title(sprintf('The %d measurements used in the calculation', matrix_counter));
    hold off;