function goalPose = GetGoalPoint(Vehicle, wall, x, d)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   HOME EXAM PART - Telerobotics and applied sensor fusion, 7.5hp
%   Author: Henrik Söderlund
%   Date: 2018-01-19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    dist = wall(1);
    ang = wall(2);
    R = [cos(ang) -sin(ang); 
         sin(ang) cos(ang)];
    v_0 = R*[1; 0];
    v_offset = R*[1; 0].*wall(1);
    R90 = [cos(pi/2) -sin(pi/2); 
         sin(pi/2) cos(pi/2)];
    v_1 = R90*v_0;

    dir = sign(dist - Vehicle.pose(1)*cos(ang) - Vehicle.pose(2)*sin(ang)); % Wu Zu Yu

    goalPosition = v_offset + v_1.*x -dir*v_0.*d;
    goalOrientation = ang + sign(x)*(pi/2);
    
    goalPose = [goalPosition; goalOrientation];
        
end