function PlotLaser(Vehicle, range, angle, str)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   HOME EXAM PART - Telerobotics and applied sensor fusion, 7.5hp
%   Author: Henrik Söderlund
%   Date: 2018-01-19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    ang_v = Vehicle.pose(3);
    R_v = [cos(ang_v) -sin(ang_v); 
           sin(ang_v) cos(ang_v)];
    sensor_pos = Vehicle.pose(1:2) + R_v*Vehicle.relSensorPos;
    laser_x = [sensor_pos(1), sensor_pos(1)+range*cos(angle)];
    laser_y = [sensor_pos(2), sensor_pos(2)+range*sin(angle)];
    
    plot(laser_x, laser_y, str);

end