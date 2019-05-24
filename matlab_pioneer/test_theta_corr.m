%% Create map
clear all;
close all;
clf;
[signalclose, Fs]=audioread('door_closed.mp3');
[signalshutdown, Fs1]=audioread('shutdown.mp3');
%lidar=SetupLidar();
image = imread('floor_plant_1.jpg');
% Crop image to relevant area
imageCropped = image(1:1150,1:1150);
image = imageCropped < 100;
% create occupancy grid. Free space is 0, for occupied space is 1
% .yaml: Resolution of the map, meters / pixel = 0.020000
% . BinaryOccupancyGrid: resoultion of cells per meter
map = robotics.BinaryOccupancyGrid(image, 50);

% Copy and inflate the map to factor in the robot's size for obstacle 
% avoidance. Setting higher to get trajectory in middle of halway.
robotRadius = 0.15 %0.2; %0.1
mapInflated = copy(map);
inflate(mapInflated, robotRadius);
show(mapInflated)
hold on;

 % TUNING VARIABLES
 radius = 0.25;
 measurment_points = 30;

%path = path_planner();
 
% INTERPOLATION AND PLOTS

path = dlmread('path_theta_test.txt');

x = path(:,1)';
y = path(:,2)';

corr_points = [21.53 8.4; 21.53 6.4];

% Make sure elements are distinct for interpolating
for i = 1:length(x)
    for j = 1:length(x) 
        if (x(i) == x(j) && i~=j)
            x(j) = x(j) + 0.0001;
        end
        if (y(i) == y(j) && i~=j)
            y(j) = y(j) + 0.0001;
        end
    end
end

p = pchip(x,y);
t = linspace(0,1000,length(x));
xq = linspace(0,1000,measurment_points);
% returns a piecewise polynomial structure
ppx = pchip(t,x);
ppy = pchip(t,y);
% evaluates the piecewise polynomial pp at the query points xq
x_ref = ppval(ppx, xq);
y_ref = ppval(ppy, xq);

% Plotting reference trajectory in node-map
figure(1);
plot(x,y,'o',x_ref,y_ref,'-','LineWidth',2);
plot(corr_points(:,1), corr_points(:,2), '+');

corr_points_x = corr_points(:,1) - x(1);
corr_points_y = corr_points(:,2) - y(1);

% Start in (0,0)
x = x - x(1);
y = y - y(1);
x_ref = x_ref - x_ref(1);
y_ref = y_ref - y_ref(1);

% Rotate points
theta = atan2( (y_ref(2) - y_ref(1)) , (x_ref(2) - x_ref(1) ));
% theta = theta + 
R = [cos(-theta) -sin(-theta); sin(-theta) cos(-theta)];
trajectory_rotated = R*[x_ref ; y_ref];

corr_points_rotated = R*[corr_points_x'; corr_points_y'];

corr_points = corr_points_rotated';

% Rotate interploation points
interp_roateted = R*[x ; y];
% pick out the vectors of rotated x- and y-data
x_ref = trajectory_rotated(1,:);
y_ref = trajectory_rotated(2,:);
x = interp_roateted(1,:);
y = interp_roateted(2,:);

% Calculating theta_ref
theta_ref = zeros(1,length(x_ref));
for i = 1: length(x_ref)-1 
    theta_ref(i) = atan2( (y_ref(i+1) - y_ref(i)), (x_ref(i+1) - x_ref(i) ));
end
% Setting last element to previous angle.
theta_ref(length(x_ref)) = theta_ref(length(x_ref)-1);
theta_ref = theta_ref - theta_ref(1);

trajectory_plot = figure(2);
axis([min(x_ref)-1, max(x_ref)+1,min(y_ref)-2,max(y_ref)+2])
gg = plot(x_ref,y_ref,'o',x_ref,y_ref,'-',corr_points(:,1),corr_points(:,2),'*','LineWidth',2);
title('TRAJECTORY')
hl=legend('$Interpolation points (x,y)$' ,'$(x_{ref},y_{ref})$','$corrpoints$',  'AutoUpdate','off');
set(hl,'Interpreter','latex')
set(gg,"LineWidth",1.5)
gg=xlabel("x - [m]");
set(gg,"Fontsize",14);
gg=ylabel("y - [m]");
set(gg,"Fontsize",14);
hold on;

%% POSITION TRACKING

% start pos = [x, y, theta]
pose(1,:) = [x_ref(1), y_ref(1), theta_ref(1)];

% observed position
%pose_obs = zeros(100000, 3);
pose_obs(1,:) = pose(1,:);

% Data for plotting, if needed
e = [];
thet = [];
alpha = [];
v = [];
w = [];
a=0;

% iteration counter
k = 1;

% In Hz
r = robotics.Rate(20);

sp = serial_port_start();
%CONFIG: timer_period = 0.1. Can change to lower maybe?
pioneer_init(sp);
lidar = SetupLidar();

pause(2);


last_odom_door = [0, 0]; 
odom_door = [0, 0];
distance_to_wall = 0;
last_distance_to_wall = 0;

% door_front = [5.02, 18.36; 19.32, 4.75];
% 
% door_front_1 = [4.95, 17.4];
% door_front_3 = [19.32, 4.75];


for k1 = 1:length(x_ref)
    
    % Changing reference
    pose_ref = [x_ref(k1),y_ref(k1), theta_ref(k1)];
    
    while norm(pose_obs(k,1:2) - pose_ref(1:2)) > radius
        
        % this will be performed every dadada seconds
        % data = [pose_new, e, phi, alpha, v, w]
        data = loop(sp, pose_ref);
        
        pose_obs(k+1,:) = data(1:3);


        if ( norm(data(1:2) - corr_points(1,:)) < 0.3)
            scan = LidarScan(lidar);
            side=1;
            [distance_to_wall_1]=distance_calc(scan,side);
            odom_1=data(1:2);
        %elseif ( norm(data(1:2) - corr_points(4,:)) < 0.3)    
        elseif (norm(data(1:2) - corr_points(2,:)) < 0.3)
            scan = LidarScan(lidar);
            side=1;
            [distance_to_wall_2]=distance_calc(scan,side);
            odom_2=data(1:2);
            distance = norm(odom_1 - odom_2);
            delta = distance_to_wall_2 - distance_to_wall_1;
            % test
            theta_error = -atan(delta/distance);

            % Rotate points
            R = [cos(-theta_error) -sin(-theta_error); sin(-theta_error) cos(-theta_error)];
            trajectory_rotated = R*[x_ref - odom_1(1) ; y_ref - odom_1(2)];
            
            % test
            corr_points = R*corr_points';
            corr_points = corr_points';

            % pick out the vectors of rotated x- and y-data
            x_ref = trajectory_rotated(1,:) + odom_1(1);
            y_ref = trajectory_rotated(2,:) + odom_1(2);
            % Correcting theta_ref
            theta_ref = zeros(1,length(x_ref));
            for i = 1: length(x_ref)-1 
                theta_ref(i) = atan2( (y_ref(i+1) - y_ref(i)), (x_ref(i+1) - x_ref(i)));
            end
            % Setting last element to previous angle.
            theta_ref(length(x_ref)) = theta_ref(length(x_ref)-1);
            theta_ref = theta_ref - theta_ref(1);
            
            
            trajectory_plot = figure(2);
            gg = plot(x_ref,y_ref,'-',corr_points(:,2),corr_points(:,2),'*','LineWidth',2);
            
        end
        
    
            
        % Correcting x/y on path
        %[pose_ref,x_ref,y_ref,doors, corr_points, door_front] = path_door_correction(d_i,pose_ref, x_ref, y_ref, doors, error, corr_points, door_front);
            
        
        figure(2);
        hold on;
        plot(pose_obs(k+1,1), pose_obs(k+1,2), 'm.');
        drawnow;
        hold off;
        
        k=k+1;
        %disp(['iteration',num2str(k)])
        
        waitfor(r);
    end
end
% 
% figure(2)
% plot(pose_obs(:,1), pose_obs(:,2), 'g.')
soundsc(signalshutdown,Fs1);
pioneer_set_controls(sp, 0, 0);
pioneer_close(sp);
fclose(lidar);
stats = statistics(r)

function data = loop(sp, pose_ref)
    
    % TUNING
    K1 = 0.41; % Artikkel: 0.41 2.94 1.42 0.5
    K2 = 2.3;%2.3;
    K3 = 1.6; %1.5;
    v_max = 1.1;
    
    
%     offset_x = 

    % READ ODOMETRY HERE to get pose_obs
    pose_obs = pioneer_read_odometry(); % offset
%     pose_obs(1)= pose(1)+ offset_x
    %convert to meter from mm and robots angular
    pose_obs(1) = pose_obs(1)/1000;
    pose_obs(2) = pose_obs(2)/1000;
    if( pose_obs(3) <= 2048)
        pose_obs(3) = pose_obs(3) * (pi / 2048);
    else
        pose_obs(3) = -(4096 - pose_obs(3)) * (pi / 2048);
    end

    e = norm(pose_ref(1:2) - pose_obs(1:2));
    theta = atan2(pose_ref(2) - pose_obs(2), pose_ref(1) - pose_obs(1)) - pose_ref(3);
    alpha = theta - pose_obs(3) + pose_ref(3);

    % Compensating for if angle is more than pi or less than pi
    if alpha > pi
        alpha = alpha - 2*pi;
    elseif alpha < -pi
        alpha = alpha + 2*pi;
    end
    
     if theta > pi
         theta = theta - 2*pi;
     elseif theta < -pi
         theta = theta + 2*pi;
     end

    % Control law
    v = v_max*tanh(K1*e);
    w = v_max*( (1+K2*(theta/alpha)) * (tanh(K1*e)/e) * sin(alpha) + K3*tanh(alpha));
    if isnan(w)
        w = 0;
    elseif isnan(v)
        v = 0;
    end
    
    % SET v AND w here
     pioneer_set_controls(sp, round(v*1000), round(w*(180/pi)))

    % ROBOT
    pose_new = pose_obs;

    if(pose_new(3)>pi)
        pose_new(3)=pose_new(3)-2*pi;
    elseif (pose_new(3)<-pi)
        pose_new(3)=pose_new(3)+2*pi;
    end
    
    data = [pose_new, e, theta, alpha, v, w];
    
end
