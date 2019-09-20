%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   DevCar Simulation for Robot SM 2019
%
%   PROGRAMMING by Henrik Söderlund (henrik.soderlund@devport.se),
%                  John Doe (foo@bar.com)
%   2019-09-11 Main script created and structured. Code inherited from
%              previous work at university.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all, close all, clc

% Define the maximum number of epochs to run the simulation 
% (for preallocation of memory)
maxEpochs = 1000;

% The default distance to drive along the wall
x = 20;
% The default distance to to keep from the wall
d = 3;

% Distance and angle to wall
wallPose1 = [8; pi/6];
wallPose2 = [8; pi/6+pi];


% Vehicle parameters
vehicleLength = 3;                  % Vehicle length between wheel axes [m]
vehicleWidth = 2;                   % Vehicle width between wheels [m]
wheelRadius = 0.35;                 % Wheel radius [m]
steeringLimits = [-1, 1].*(60*pi/180); % Steering limits [rad]
maxSpeed = 3;                       % Maximum linear velocity [m/s]

% Define starting configuration of robot
startPose = [0; 0; 0]; % Start pose (x,y,theta)
startSpeed = 0; % Start speed [rad/s]
startSteering = 0; % Start steering [rad]

% Minimum distance to stop the vehicle in front of the goal
stopDistance = 0.2;

% Lidar Sensor poses relative to the robot (x, y, theta)
lidars_relSensorPos = [3,3; 0.5, -0.5; pi/6,-pi/6];

% Ultrasound Sensor pose relative to the robot (x, y, theta)
ultra_relSensorPos = [0; 0; 0];

% Lidar maximum range [m]
lidarRange = 14;

% Define the robot struct
Vehicle = struct('pose',             startPose, ...
                 'wheelSpeed_Left',  startSpeed, ...
                 'wheelSpeed_Right', startSpeed, ...
                 'steering',         startSteering, ...
                 'speed',            startSpeed, ...
                 'maxSpeed',         maxSpeed/wheelRadius, ...
                 'steeringLimits',   steeringLimits, ...
                 'wheelRadius',      wheelRadius, ...
                 'vehicleWidth',     vehicleWidth, ...
                 'vehicleLength',    vehicleLength, ...
                 'trajectory',       zeros(2,maxEpochs), ...
                 'lidarRange',       lidarRange, ...
                 'lidars_relSensorPos', lidars_relSensorPos, ...
                 'ultra_relSensorPos',  ultra_relSensorPos);
             
% Define alignment stage variable (used with the pose controller)
align = true;

% Define logging vectors
speedHistory = zeros(2,maxEpochs);
sensorHistory = zeros(2,maxEpochs);
goalPoseHistory = zeros(3, maxEpochs);
goalEstPoseHistory = zeros(3, maxEpochs);
MahalHistory = zeros(1, maxEpochs);

% Get ground truth of wall
wallTrue1 = groundTruthSensor(Vehicle, wallPose1);
wallTrue2 = groundTruthSensor(Vehicle, wallPose2);

% Plot the first visual of the simulation
fig = figure(1);
hold on;
PlotQuadVehicle(Vehicle.pose(1),Vehicle.pose(2),Vehicle.pose(3),Vehicle.steering);
PlotHoughLine(wallTrue1(1), wallTrue1(2), 'r');
PlotHoughLine(wallTrue2(1), wallTrue2(2), 'r');
axis equal

% Define input variable
str = '';

% Define delta time
dt = 0.1;
% Define epoch variable
epoch = 1;
% Begin simulation
while(~strcmp(str,'exit'))

    % Get the variables needed
    prompt = 'Enter goal X-coordinate:';
    x = str2double(input(prompt,'s'));
    prompt = 'Enter goal Y-coordinate:';
    y = str2double(input(prompt,'s'));
    prompt = 'Enter goal angle of approach: ';
    ang = str2double(input(prompt,'s'));
    
    % Get the current state of the laser sensor
    [range1_1, range2_1] = fixedLidarSensors(Vehicle, wallPose1);
    [range1_2, range2_2] = fixedLidarSensors(Vehicle, wallPose2);
    
    % TODO: Add Ultrasound sensor
    
    % Compute a goal point along the wall using the user defined
    % variables
    %goalPose = GetGoalPoint(Vehicle, wall, x, d);
    goalPose = [x; y; ang];
    % Compute the new states to be used by the controller
    states = ComputeNewStates(Vehicle, goalPose);
    
    % Define the observation covariance matrix and the goal pose for 
    % both observation and estimation
    %C = eye(3).*0.1;
    c = goalPose;
    s = goalPose;
    
    % Drive to goal point
    while (epoch < maxEpochs) % distanceDriven < (abs(x)-stopDistance) && 
        % Increment epoch
        epoch = epoch + 1;
        
        % Get ground truth
        wallTrue1 = groundTruthSensor(Vehicle, wallPose1);
        wallTrue2 = groundTruthSensor(Vehicle, wallPose2);
        % Get the current state of the laser sensor
        [range1_1, range2_1] = fixedLidarSensors(Vehicle, wallPose1);
        [range1_2, range2_2] = fixedLidarSensors(Vehicle, wallPose2);
        
        % TODO: Add Ultrasound sensor
        
        % Compute a goal point along the wall using the user defined
        % variables
        %goalPose = GetGoalPoint(Vehicle, wall, x, d);
        % Compute the new states to be used by the controller
        states = ComputeNewStates(Vehicle, s);
        
        % Plot the current state in the simulation
        fig = figure(1);
        % Clear
        clf
        hold on;
        % Plot vehicle trajectory
        plot(Vehicle.trajectory(1,1:(epoch-1)), Vehicle.trajectory(2,1:(epoch-1)), 'k-');
        % Plot vehicle pose
        PlotQuadVehicle(Vehicle.pose(1),Vehicle.pose(2),Vehicle.pose(3),Vehicle.steering);
        % Plot wall ground truth
        PlotHoughLine(wallTrue1(1), wallTrue1(2), 'r');
        PlotHoughLine(wallTrue2(1), wallTrue2(2), 'r');
        % Plot laser hit
        %PlotLaser(Vehicle, range, angle, 'g--');
        % Plot line from vehicle to goal
        plot([Vehicle.pose(1), goalPose(1)],[Vehicle.pose(2) goalPose(2)],'b--');
        % Plot observed goal point
        plot(goalPose(1), goalPose(2), 'b*');
        % Plot observed goal orientation
        quiver(goalPose(1), goalPose(2),2*cos(goalPose(3)),2*sin(goalPose(3)), 'b');
        % Plot covariance ellipse
        %if (sum(eig(C) > 0) == numel(eig(C)))
        %    h1=error_ellipse(C(1:2,1:2), s(1:2));
        %end
        % Plot estimated goal point
        plot(s(1), s(2), 'r*');
        % Plot estimated goal orientation
        quiver(s(1), s(2),2*cos(s(3)),2*sin(s(3)), 'r');
        % Plot settings
        xlabel('World x [m]');
        ylabel('World y [m]');
        axis equal
        % Draw
        drawnow
        
        % Update dead-reckoning of the vehicle
        Vehicle = DeadReckoning(Vehicle, dt);
        
        % Store trajectory in vehicle struct
        Vehicle.trajectory(:,epoch) = Vehicle.pose(1:2);
        
        % Compute controller 1 output from current states
        [v, gamma1] = PoseController(states, align);
        % Compute controller 2 output from current states
        %gamma2 = WallController(states, Vehicle, wall, d);
        gamma2 = 0;
        % Combine controllers
        gamma = gamma1 + gamma2;
        
        % Determine if the vehicle if facing the goal, otherwise we
        % have to align it
        if (states(2) <= pi/2 && states(2) > -pi/2)
            align = false;
        end

        % Limit the linear velocity
        v = min(v, Vehicle.maxSpeed);
        % Limit the steering angle
        gamma = clamp(gamma,Vehicle.steeringLimits);

        % Compute applied wheel angular velocities
        [w_r, w_l] = ComputeWheelSpeeds(Vehicle, v, gamma);

        % Apply wheel angular velocities
        Vehicle.wheelSpeed_Right = w_r;
        Vehicle.wheelSpeed_Left = w_l;

        % Update states for controllers
        states = UpdateState(states, Vehicle , dt);

        % Store logging data
        speedHistory(:,epoch) = [w_r; w_l];
        %sensorHistory(:,epoch) = [range; angle];
        goalPoseHistory(:,epoch) = goalPose;
        goalEstPoseHistory(:,epoch) = s;

        % Compute covariance matrix of observed goal points
        %C = CovarianceMatrix(goalPoseHistory(:,1:epoch));
        c = goalPose;

        % Store mahalanobis distance history
        %MahalHistory(epoch) = (s-c)'*inv(C)*(s-c);

        % Fuse current observation with estimated goal point
        s = SensorFusion(c, s, 0.1);

    end
    
    % Get the time vector
    timeVector = dt.*(0:(epoch-1));

    % Plot speed history
    fig = figure(2);
    hold on;
    grid on;
    plot(timeVector, speedHistory(1,1:epoch), 'r');
    plot(timeVector, speedHistory(2,1:epoch), 'b');
    xlabel('time [s]');
    ylabel('Wheel speed [rad/s]');
    legend('Right wheel', 'Left wheel');

    % Plot sensor history
    fig = figure(3);
    hold on;
    grid on;
    plot(timeVector, sensorHistory(1,1:epoch), 'r');
    plot(timeVector, sensorHistory(2,1:epoch), 'b');
    xlabel('time [s]');
    ylabel('Sensor values');
    legend('Sensor range [m]', 'Sensor angle [rad]');

    % Plot Goal pose history
    fig = figure(4);
    hold on;
    grid on;
    plot(timeVector, goalPoseHistory(:,1:epoch), '--');
    plot(timeVector, goalEstPoseHistory(:,1:epoch), '-');
    xlabel('time [s]');
    ylabel('Goal Poses');
    legend('Observation goal x [m]', 'Observation goal y [m]', ...
        'Observation goal angle [rad]', 'Estimation goal x [m]', ...
        'Estimation goal y [rad]', 'Estimation goal angle [rad]');

    % Plot Mahalanobis distance history
    fig = figure(5);
    hold on;
    grid on;
    plot(timeVector, MahalHistory(1:epoch), 'r');
    xlabel('time [s]');
    ylabel('Mahalanobis distance');
    yaxis([-40,40]);

end