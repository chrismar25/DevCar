function [wall, distanceDriven] = SensorToWorldWall(Vehicle, range, angle)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   HOME EXAM PART - Telerobotics and applied sensor fusion, 7.5hp
%   Author: Henrik Söderlund
%   Date: 2018-01-19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    a = Vehicle.pose(1:2);
    b = [cos(angle); sin(angle)];
    proj_x = dot(a, b);

    wall = [range + proj_x; angle];
    distanceDriven = norm(a - proj_x);

end