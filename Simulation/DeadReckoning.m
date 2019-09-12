function Vehicle = DeadReckoning(Vehicle, dt)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   HOME EXAM PART - Telerobotics and applied sensor fusion, 7.5hp
%   Author: Henrik Söderlund
%   Date: 2018-01-19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Handbook of robotics
    
    w_r = Vehicle.wheelSpeed_Right;
    w_l = Vehicle.wheelSpeed_Left;
    
    v = (Vehicle.wheelRadius/2) * (w_r + w_l);
    omega = (Vehicle.wheelRadius/Vehicle.vehicleWidth) * (w_r - w_l);

    omega = clamp(omega,Vehicle.steeringLimits);
    
    theta = Vehicle.pose(3);
    
    model = [v*cos(theta); v*sin(theta); omega];
    
    Vehicle.pose = Vehicle.pose + dt.*model;
    Vehicle.steering = omega;
    Vehicle.speed = v;
    %disp(Vehicle.pose)
end