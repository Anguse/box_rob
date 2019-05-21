function [ddx, ddy, dda, C] = Cox_LineFit_h(angs, meas, p_state, sensor_offset, LINEMODEL)

% Setup
alpha = sensor_offset(1);  % [Scalar] Offset between sensors centre and robots x-axis
beta = sensor_offset(2);   % [Scalar] Offset between sensors centre and robots y-axis
gamma = sensor_offset(3);  % [Scalar] Heading offset between sensors and robots x-axis(-pi/2 radians)
vm = [p_state(1) p_state(2)]'; %mean([meas.*cos(angs) meas.*sin(angs)]); % Mean of all points
dx = 0;
dy = 0;
da = 0;
ddx = 0; ddy = 0; dda = 0;
Xr = p_state;



% Get unit vectors of all lines
rot = [0 -1;1 0]; % 90 degree rotation matrix
for kk = 1:size(LINEMODEL, 1)
    L1 = [LINEMODEL(kk,1) LINEMODEL(kk,3); LINEMODEL(kk,2) LINEMODEL(kk,4)]; % [X1, Y1; X2, Y2]
    L2 = [LINEMODEL(kk,1), LINEMODEL(kk,1);      % [X1, Y1; X1, Y1]
          LINEMODEL(kk,2), LINEMODEL(kk,2)]; 
   % L1 = [LINEMODEL(kk,1) LINEMODEL(kk,2); LINEMODEL(kk,3) LINEMODEL(kk,4)]; % [X1, Y1; X2, Y2]
   % L2 = [LINEMODEL(kk,1), LINEMODEL(kk,2);      % [X1, Y1; X1, Y1]
   %       LINEMODEL(kk,1), LINEMODEL(kk,2)];  
      
    V = rot*(L1-L2);       % rot*[0,  0; X2-X1, Y2-Y1] Rotate line 90 degrees and move it to origo 
    Ui = V/norm(V);         % Normalize for unitvector
    Ui2 = Ui;
    Ui = [Ui(3) Ui(4)];     % [Xu Yu]
    z = [L1(3) L1(4)]; %[(L1(1)+L1(3))/2 (L1(2)+L1(4))/2];     % A point on L1
    Ri = dot(Ui, z);       % Project unitvector on L1
    U(kk,1:2) = Ui;        
    R(kk,1) = Ri;
    %line([L1(1) L1(3)], [L1(2) L1(4)])
end

% The loop
iterations = 0;
finished = false;
while ~finished  
    inliers = 0;
    iterations = iterations + 1;
    % Get meadian value of all target distances
    COLOR_MAP = hot(size(angs, 2));
    for ii = 1:size(angs, 2)
        x = meas(ii)*cos(angs(ii));
        y = meas(ii)*sin(angs(ii));
        rot = [cos(gamma) -sin(gamma) alpha; sin(gamma) cos(gamma) beta; 0 0 1];
        Xs = rot*[x y 1]';
        rot = [cos(Xr(3)) -sin(Xr(3)) Xr(1); sin(Xr(3)) cos(Xr(3)) Xr(2); 0 0 1];
        Xw = rot*[Xs(1) Xs(2) 1]';
        v = [Xw(1), Xw(2)]';
        for kk = 1:size(U, 1)
            yy(kk) = abs(R(kk) - dot(U(kk,1:2), v));
        end
        [targ(ii), targind] = min(yy);
    end
    threshold = median(targ);
    
    for kk = 1:size(angs, 2)
        
        % 1.Translate and rotate data points
        x = meas(kk)*cos(angs(kk));
        y = meas(kk)*sin(angs(kk));
        rot = [cos(gamma) -sin(gamma) alpha; sin(gamma) cos(gamma) beta; 0 0 1];
        Xs = rot*[x y 1]';
        rot = [cos(Xr(3)) -sin(Xr(3)) Xr(1); sin(Xr(3)) cos(Xr(3)) Xr(2); 0 0 1];
        Xw = rot*[Xs(1) Xs(2) 1]';
        v = [Xw(1), Xw(2)]';
        
        % 2.Find the target line
        for ii = 1:size(U, 1)
            yi(ii) = R(ii) - dot(U(ii,1:2), v);
        end
        [target, target_index] = min(abs(yi));
        if target < threshold % Reject all outliers
            inliers = inliers + 1;
            X1(inliers,1) = U(target_index, 1);
            X2(inliers,1) = U(target_index, 2);
            X3(inliers,1) = U(target_index, :)*[0 -1;1 0]*(v-vm);
            targets(inliers,1) = yi(target_index); %Witho sign
            vi(inliers, 1:2) = v;
            vi(inliers, 3) = target_index;
        end
    end
    
    % Too few inliers, increase threshold
    if inliers < 5
        if threshold > 300
            ddx = 0;
            ddy = 0;
            dda = 0;
            C = [1 0 0;
                 0 1 0;
                 0 0 1];
            return;
        end
        if(iterations > 100)
            ddx = 0;
            ddy = 0;
            dda = 0;
            C = [1 0 0;
                 0 1 0;
                 0 0 1];
            finished = true;
            break;
        end
        threshold = threshold + 1;
        continue;
    end
    
    % 3.Set up linear equation system
    A = [X1 X2 X3];
    B =  A\targets;

    S2 = ((targets-A*B)'*(targets-A*B))/(max(size(A))-4);
    C = S2*(inv(A'*A));

    % 4.Add latest contribution to the overall congruence
    dx = B(1);
    dy = B(2);
    da = B(3);
    
    ddx = ddx + dx;
    ddy = ddy + dy;
    dda = dda + da;

    % Change the position
    Xr(1) = Xr(1)+dx;
    Xr(2) = Xr(2)+dy;
    Xr(3) = Xr(3)+da;
    
    % 5.Check if process has converged
    if(sqrt(dx^2+dy^2) < 15)&&(abs(da<0.1*pi/180))
        finished = true;
        break;
    end
    
    if(iterations>100)
        ddx = 0;
        ddy = 0;
        dda = 0;
        C = [1 0 0;
                 0 1 0;
                 0 0 1];
        finished = true;
        break;
    end
   
  %  clear X1;
  %  clear X2;
  %  clear X3;
  %  clear vi;
    targets = 0;
end


