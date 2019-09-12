function gamma = ObstacleAvoidance(Vehicle, wall, gamma)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   HOME EXAM PART - Telerobotics and applied sensor fusion, 7.5hp
%   Author: Henrik Söderlund
%   Date: 2018-01-19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    avoidanceGain = 0.2;

    a = Vehicle.pose(1:2);
    b = wall(1).*[cos(wall(2)); sin(wall(2))];
    proj_x = dot(a, (b/norm(b)));
    
    distToWall = norm(b) - proj_x;
    angToWall = wall(2);

    if (abs(angToWall) <= pi)
        avoidance = -sign(angToWall)*(cos(angToWall) + abs(sin(angToWall))) ...
            * ((Vehicle.vehicleLength)/distToWall);
        gamma = gamma - avoidanceGain*avoidance;
    end

end