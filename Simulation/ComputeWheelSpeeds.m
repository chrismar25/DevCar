function [w_r, w_l] = ComputeWheelSpeeds(Vehicle, v, omega)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   HOME EXAM PART - Telerobotics and applied sensor fusion, 7.5hp
%   Author: Henrik Söderlund
%   Date: 2018-01-19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Handbook of robotics
    
    theta = Vehicle.pose(3);
    linearVel = [v*cos(theta); v*sin(theta); omega];
    
    R = Vehicle.wheelRadius;
    L = Vehicle.vehicleWidth;
    
    C = [(R/2)*cos(theta), (R/2)*cos(theta);
         (R/2)*sin(theta), (R/2)*sin(theta);
         R/L, -R/L];
    
    w = C\linearVel;
    
    w_r = w(1);
    w_l = w(2);
end