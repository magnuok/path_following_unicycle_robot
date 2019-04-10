
path = [2.00    1.00;
        1.25    1.75;
        5.25    8.25;
        7.25    8.75;
        11.75   10.75;
        12.00   10.00];
    
robotCurrentLocation = path(1,:);
robotGoal = path(end,:);
initialOrientation = 0;
robotCurrentPose = [robotCurrentLocation initialOrientation];

% SIMULATOR
robotRadius = 0.4;
robot = ExampleHelperRobotSimulator('emptyMap',2);
robot.enableLaser(false);
robot.setRobotSize(robotRadius);
robot.showTrajectory(true);
robot.setRobotPose(robotCurrentPose);

% Plot path
plot(path(:,1), path(:,2),'k--d')
xlim([0 13])
ylim([0 13])

% Define the Path Following Controller
% controller = robotics.PurePursuit;
% controller.Waypoints = path;
% controller.DesiredLinearVelocity = 0.3;
% controller.MaxAngularVelocity = 2;

% As a general rule, the lookahead distance should be larger than the 
% desired linear velocity for a smooth path. The robot might cut corners 
% when the lookahead distance is large. In contrast, a small lookahead 
% distance can result in an unstable path following behavior.
% A value of 0.5 m was chosen for this example.
% controller.LookaheadDistance = 0.5;

% Define a goal radius, which is the desired distance threshold between 
% the robot's final location and the goal location
goalRadius = 0.1;
distanceToGoal = norm(robotCurrentLocation - robotGoal);


% The robotics.PurePursuit object computes control commands for the robot. 
% The controller runs at 10 Hz.


v_ref = 0.3;
omega_ref = 0.3;

x_prev = 0;
y_prev = 0;
omega_prev = 0;

Hz = 10;

%% 
controlRate = robotics.Rate(Hz);
while( distanceToGoal > goalRadius )
    
    pose = robot.getRobotPose;
    x = pose(1);
    y = pose(2);
    omega = pose(3);
    
    % Calculate speed
    x_dot = (x - x_prev)/(1/Hz);
    y_dot = (x - x_prev)/(1/Hz);
    omega_dot = (omega- omega_prev)/(1/Hz);
    
    % Errors
    omega_err = omega - omega;
    
    omega = omega_ref + c1 * pose(3)
    
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robot.getRobotPose);
    
    % Simulate the robot using the controller outputs.
    drive(robot, v, omega);
    
    % Extract current location information ([X,Y]) from the current pose of the
    % robot
    robotCurrentPose = robot.getRobotPose;
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);
    
    waitfor(controlRate);
    
end


delete(robot)



