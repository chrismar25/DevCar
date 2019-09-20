function [range1, range2] = fixedLidarSensors(Vehicle, wall)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   HOME EXAM PART - Telerobotics and applied sensor fusion, 7.5hp
%   Author: Henrik Söderlund
%   Date: 2018-01-19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Get the relative poses of both sensors
    lidar1 = Vehicle.lidars_relSensorPos(:,1);
    lidar2 = Vehicle.lidars_relSensorPos(:,2);
    
    % Get distance to wall and angle to wall from world origin
    dist = wall(1);
    ang = wall(2);
    
    % Transform the wall vector to a line described by two points
    x1=dist*cos(ang);
    y1=dist*sin(ang);
    x2= 20*cos(ang+pi/2);
    y2=20*sin(ang+pi/2);
    x3=x1-x2;
    y3=y1-y2;
    x2=x1+x2;
    y2=y1+y2;

    %  Create rotation matrix of robot heading
    R_rob = [cos(Vehicle.pose(3)), -sin(Vehicle.pose(3));
             sin(Vehicle.pose(3)), cos(Vehicle.pose(3))];
     
    %  Position the lidars realtive to world frame
    lidar1_pos = Vehicle.pose(1:2) + R_rob*lidar1(1:2);
    lidar2_pos = Vehicle.pose(1:2) + R_rob*lidar2(1:2);
  
    % Create the pose vector of the lidars and rotate them
    lidar1_pose = [lidar1_pos; lidar1(3) + Vehicle.pose(3)];
    lidar2_pose = [lidar2_pos; lidar2(3) + Vehicle.pose(3)];
    
    % Create lidar sensor lines (laser beams)
    lidar1_line = [lidar1_pose(1) lidar1_pose(1)+Vehicle.lidarRange*cos(lidar1_pose(3)); ...
                   lidar1_pose(2) lidar1_pose(2)+Vehicle.lidarRange*sin(lidar1_pose(3))];
    lidar2_line = [lidar2_pose(1) lidar2_pose(1)+Vehicle.lidarRange*cos(lidar2_pose(3)); ...
                   lidar2_pose(2) lidar2_pose(2)+Vehicle.lidarRange*sin(lidar2_pose(3))];
    
    % FIn intersection between lidar "beams" and wall
    [x1_x, y1_x, flag1] = findIntersection(lidar1_line, [x2 x3; y2 y3]);
    [x2_x, y2_x, flag2] = findIntersection(lidar2_line, [x2 x3; y2 y3]);
    
    % TODO: REMOVE POINT BEHIND
%     if (flag1 ~= -1)
%         onLine1 = isPointOnLine(lidar1_line(:,1)', lidar1_line(:,2)', [x1_x; y1_x]');
%         if onLine1 ~= 1
%             flag1 = -1;
%         end
%     end
%     
%     if (flag2 ~= -1)
%         onLine2 = isPointOnLine(lidar2_line(:,1)', lidar2_line(:,2)', [x2_x; y2_x]');
%         if onLine2 ~= 1
%             flag2 = -1;
%         end
%     end
    
    if (flag1 ~= -1)
        % draw line
        plot(lidar1_line(1,1), lidar1_line(2,1), 'm.', 'MarkerSize', 5);
        quiver(lidar1_line(1,1), lidar1_line(2,1),2*cos(lidar1(3)+Vehicle.pose(3)),2*sin(lidar1(3)+Vehicle.pose(3)), 'm');
        plot(x1_x, y1_x, 'ms');

        % Compute range
        range1 = norm([x1_x; y1_x] - lidar1_line(:,1)) + (0.2)*randn;
    else
        range1 = Vehicle.lidarRange;
    end
    
    if (flag2 ~= -1)
        plot(lidar2_line(1,1), lidar2_line(2,1), 'm.', 'MarkerSize', 5);
        quiver(lidar2_line(1,1), lidar2_line(2,1),2*cos(lidar2(3)+Vehicle.pose(3)),2*sin(lidar2(3)+Vehicle.pose(3)), 'm');
        plot(x2_x, y2_x, 'ms');

        range2 = norm([x2_x; y2_x] - lidar2_line(:,1)) + (0.2)*randn;
    else
        range2 = Vehicle.lidarRange;
    end
    
    
    %  If you want...
    function R = isPointOnLine(P1, P2, Q)
        % Is point Q=[x3,y3] on line through P1=[x1,y1] and P2=[x2,y2]
        % Normal along the line:
        P12 = P2 - P1;
        L12 = sqrt(P12 * P12');
        N   = P12 / L12;
        % Line from P1 to Q:
        PQ = Q - P1;
        % Norm of distance vector: LPQ = N x PQ
        Dist = abs(N(1) * PQ(2) - N(2) * PQ(1));
        % Consider rounding errors:
        Limit = 10 * eps(max(abs(cat(1, P1(:), P2(:), Q(:)))));
        R     = (Dist < Limit);
        % Consider end points if any 4th input is used:
        if R && nargin == 4
          % Projection of the vector from P1 to Q on the line:
          L = PQ * N.';  % DOT product
          R = (L > 0.0 && L < L12);
        end
    end
    
end