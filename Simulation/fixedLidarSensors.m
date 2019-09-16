function [range1, range2] = fixedLidarSensors(Vehicle, wall)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   HOME EXAM PART - Telerobotics and applied sensor fusion, 7.5hp
%   Author: Henrik Söderlund
%   Date: 2018-01-19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    lidar1 = Vehicle.lidars_relSensorPos(:,1);
    lidar2 = Vehicle.lidars_relSensorPos(:,2);
    
    dist = wall(1);
    ang = wall(2);
    
    x1=dist*cos(ang);
    y1=dist*sin(ang);
    x2= 20*cos(ang+pi/2);
    y2=20*sin(ang+pi/2);
    x3=x1-x2;
    y3=y1-y2;
    x2=x1+x2;
    y2=y1+y2;
    
    lidar1_line = Vehicle.pose(1:2) + [lidar1(1) lidar1(1)+Vehicle.lidarRange*cos(lidar1(3)+Vehicle.pose(3)); ...
                                       lidar1(2) lidar1(2)+Vehicle.lidarRange*sin(lidar1(3)+Vehicle.pose(3))];
               
    lidar2_line = Vehicle.pose(1:2) + [lidar2(1) lidar2(1)+Vehicle.lidarRange*cos(lidar2(3)+Vehicle.pose(3)); ...
                                       lidar2(2) lidar2(2)+Vehicle.lidarRange*sin(lidar2(3)+Vehicle.pose(3))];
    
    [x1_x, y1_x, ~] = findIntersection(lidar1_line, [x2 x3; y2 y3]);
    [x2_x, y2_x, ~] = findIntersection(lidar2_line, [x2 x3; y2 y3]);
    
    plot(lidar1_line(1,1), lidar1_line(2,1), 'b*');
    plot(lidar2_line(1,1), lidar2_line(2,1), 'r*');
    
    quiver(lidar1_line(1,1), lidar1_line(2,1),2*cos(lidar1(3)+Vehicle.pose(3)),2*sin(lidar1(3)+Vehicle.pose(3)), 'b');
    quiver(lidar2_line(1,1), lidar2_line(2,1),2*cos(lidar2(3)+Vehicle.pose(3)),2*sin(lidar2(3)+Vehicle.pose(3)), 'r');
    
    plot(x1_x, y1_x, 'bs');
    plot(x2_x, y2_x, 'rs');

    range1 = norm([x1_x; y1_x] - lidar1_line(:,1)) + (0.2)*randn;
    range2 = norm([x2_x; y2_x] - lidar2_line(:,1)) + (0.2)*randn;
    
end