function [wall, distanceDriven] = groundTruthSensor(Vehicle, wall)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   HOME EXAM PART - Telerobotics and applied sensor fusion, 7.5hp
%   Author: Henrik S�derlund
%   Date: 2018-01-19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    angle = wall(2);
    a = Vehicle.pose(1:2);
    b = [cos(angle); sin(angle)];
    proj_x = dot(a, b);
    
    distanceDriven = norm(a - proj_x);

end