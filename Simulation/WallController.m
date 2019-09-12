function gamma = WallController(states, Vehicle, wall, setpoint)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   HOME EXAM PART - Telerobotics and applied sensor fusion, 7.5hp
%   Author: Henrik Söderlund
%   Date: 2018-01-19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    k_gamma = 0.1;
    
    a = Vehicle.pose(1:2);
    b = wall(1).*[cos(wall(2)); sin(wall(2))];
    proj_x = dot(a, (b/norm(b)));
    
    distToWall = norm(b) - proj_x;
    error = setpoint - distToWall;
    
    alpha = states(2);
    if (alpha < 0)
        gamma = -k_gamma*error;
    elseif (alpha > 0)
        gamma = k_gamma*error;
    else
        gamma = 0;
    end
end