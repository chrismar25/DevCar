function [wall, distanceDriven] = groundTruthSensor(Vehicle)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   HOME EXAM PART - Telerobotics and applied sensor fusion, 7.5hp
%   Author: Henrik Söderlund
%   Date: 2018-01-19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    wall = [8; pi/6];
    
    angle = wall(2);
    a = Vehicle.pose(1:2);
    b = [cos(angle); sin(angle)];
    proj_x = dot(a, b);
    
    distanceDriven = norm(a - proj_x);

end