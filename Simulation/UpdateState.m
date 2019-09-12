function states = UpdateState(states, Vehicle, dt)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   HOME EXAM PART - Telerobotics and applied sensor fusion, 7.5hp
%   Author: Henrik Söderlund
%   Date: 2018-01-19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Handbook of robotics
    rho = states(1);
    alpha = states(2);

    %if (alpha <= pi/2 && alpha > -pi/2)
        v = Vehicle.speed;
        gamma = Vehicle.steering;
        gamma = clamp(gamma,Vehicle.steeringLimits);

        model = [-cos(alpha),     0; 
                 sin(alpha)/rho, -1; 
                 -sin(alpha)/rho, 0] * [v; gamma];

        states = states + dt.*model;
        %disp(states)
    %end
end