function states = ComputeNewStates(Vehicle, goalPose)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   HOME EXAM PART - Telerobotics and applied sensor fusion, 7.5hp
%   Author: Henrik Söderlund
%   Date: 2018-01-19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    rho = norm(goalPose(1:2) - Vehicle.pose(1:2));
    alpha = atan2((goalPose(2) - Vehicle.pose(2)), ...
                  (goalPose(1) - Vehicle.pose(1))) - Vehicle.pose(3);
    beta = - Vehicle.pose(3) - alpha + goalPose(3);

    states = [rho; alpha; beta]; % rho, alpha, beta
    
end