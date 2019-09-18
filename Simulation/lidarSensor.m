function [range, angle] = lidarSensor(Vehicle, wall)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   HOME EXAM PART - Telerobotics and applied sensor fusion, 7.5hp
%   Author: Henrik Söderlund
%   Date: 2018-01-19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    dist = wall(1);
    ang = wall(2);
    R = [cos(ang) -sin(ang); 
         sin(ang) cos(ang)];
    wall_dirTo = R*[1; 0];
    wall_pos = R*[1; 0].*dist;
    R90 = [cos(pi/2) -sin(pi/2); 
         sin(pi/2) cos(pi/2)];
    wall_dir = R90*wall_dirTo;
    
    ang_v = Vehicle.pose(3);
    R_v = [cos(ang_v) -sin(ang_v); 
           sin(ang_v) cos(ang_v)];
    
    sensor_pos = Vehicle.pose(1:2) + R_v*Vehicle.relSensorPos;
    
    hitPoint = wall_pos + dot(sensor_pos-wall_pos,wall_dir)*wall_dir;
    
    lidarHit = hitPoint - sensor_pos;
    
    range = norm(lidarHit);
    angle = atan2(lidarHit(2), lidarHit(1));
    
    range = range + (0.1)*randn;
    angle = angle + (5*pi/180)*randn;
    
end