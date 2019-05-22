%% Create map
clear all;
close all;
clf;
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
robotRadius = 0.25 %0.2; %0.1
mapInflated = copy(map);
inflate(mapInflated, robotRadius);
show(mapInflated)
hold on;

global doors;
doors = dlmread('doors.txt'); % [x,y,bol] bol=1 right bol=0 left

 % TUNING VARIABLES
 radius = 0.2;
 measurment_points = 250;

%path = path_planner();
 
% INTERPOLATION AND PLOTS

path = dlmread('path_nice_corrected_2.txt');

x = path(:,1)';
y = path(:,2)';

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
plot(doors(:,1)/1000, doors(:,2)/1000, '*');

% Doors
doors_x = doors(:,1) - x(1)*1000;
doors_y = doors(:,2) - y(1)*1000;

% Start in (0,0)
x = x - x(1);
y = y - y(1);
x_ref = x_ref - x_ref(1);
y_ref = y_ref - y_ref(1);

% Rotate points
theta = atan2( (y_ref(2) - y_ref(1)) , (x_ref(2) - x_ref(1) ));
R = [cos(-theta) -sin(-theta); sin(-theta) cos(-theta)];
trajectory_rotated = R*[x_ref ; y_ref];

doors_rotated = R*[doors_x' ; doors_y'];
doors(:,1:2) = doors_rotated';

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
axis([map.XWorldLimits(1),map.XWorldLimits(2),map.YWorldLimits(1),map.YWorldLimits(2)])
gg = plot(x_ref,y_ref,'o',x_ref,y_ref,'-',doors_rotated(1,:)/1000,doors_rotated(2,:)/1000,'*','LineWidth',2);
title('TRAJECTORY')
hl=legend('$Interpolation points (x,y)$' ,'$(x_{ref},y_{ref})$','$Door coordinates (x,y)$' ,  'AutoUpdate','off');
set(hl,'Interpreter','latex')
set(gg,"LineWidth",1.5)
gg=xlabel("x - [m]");
set(gg,"Fontsize",14);
gg=ylabel("y - [m]");
set(gg,"Fontsize",14);
hold on;


% Plotting reference circle around each point
% for i = 1:length(x_ref)
%     %// center
%     c = [x_ref(i) y_ref(i)];
% 
%     pos = [c-radius 2*radius 2*radius];
%     rectangle('Position',pos,'Curvature',[1 1])
%     axis equal
%     
%     text(x_ref(i) + 0.1,y_ref(i) + 0.1 ,num2str(i),'Color','k')
% end
% hold on;


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

for k1 = 1:length(x_ref)
    
    % Changing reference
    pose_ref = [x_ref(k1),y_ref(k1), theta_ref(k1)];
    
    while norm(pose_obs(k,1:2) - pose_ref(1:2)) > radius
        
        % this will be performed every dadada seconds
        % data = [pose_new, e, phi, alpha, v, w]
        data = loop(sp, pose_ref);
        
        pose_obs(k+1,:) = data(1:3);

        % Find close by doors
        %start_coordinates = [2590, 20710];
        pos = data(1:2)*1000;
        range_threshold = 1300; % Search for the door inside threshold
        nearby_door_right = [];
        nearby_door_left = [];
        door_detected = [0 0];
        
        for i = 1:length(doors(:,1))
            range = norm([doors(i,1),doors(i,2)] - pos(1:2) );
            
            % if it is close enough and not discovered.
            if range < range_threshold && doors(i,4) == 0 
                if doors(i,3) == 1
                    nearby_door_right = [doors(i,:), i];% adding index because needed to change detected or not parameter to true/false
                    d_i= i;
                else
                    nearby_door_left = [doors(i,:), i]; 
                    d_i= i;
                end
            end
        end

        
        %if close to door, search for them
        if ~isempty(nearby_door_right) || ~isempty(nearby_door_left)
            scan = LidarScan(lidar);
            door_detected = door_detector(nearby_door_right, nearby_door_left, scan);
        end
        
        % door is detected, drive to evaluate if door is open or not.
        if door_detected(1) == 1 % Door on the right side 
            
             side=-1;
            [distance_to_wall]=distance_calc(scan,side);
            [error,doors]=door_turn(doors,d_i,sp,lidar,distance_to_wall,side);

            [pose_ref,x_ref,y_ref,doors]=path_door_correction(d_i,pose_ref,x_ref,y_ref,doors,error);

        
        elseif door_detected(2) == 1 %Door on the left side   
            side=1;
            
            last_odom_door = odom_door;
            odom_door = data(1:2);
            last_distance_to_wall = distance_to_wall;
            distance_to_wall = distance_calc(scan,side);
            
            [error,doors]=door_turn(doors,d_i,sp,lidar,distance_to_wall,side);

            % Correcting rotation of x/y on path
            if (d_i == 2 || d_i == 5 || d_i == 7 || d_i == 10 || d_i == 16)
                distance = norm(odom_door - last_odom_door);
                delta = last_distance_to_wall - distance_to_wall;
                % test
                theta_error = atan(delta/distance);

                % Doors
                doors_x = doors(:,1);
                doors_y = doors(:,2);
                %x_ref = x_ref;
                %y_ref = y_ref;

                % Rotate points
                R = [cos(-theta_error) -sin(-theta_error); sin(-theta_error) cos(-theta_error)];
                trajectory_rotated = R*[x_ref ; y_ref];
                doors_rotated = R*[doors_x' ; doors_y'];

                % put doors back
                doors(:,1:2) = doors_rotated';

                % pick out the vectors of rotated x- and y-data
                x_ref = trajectory_rotated(1,:);
                y_ref = trajectory_rotated(2,:);
                
                
                % Correcting theta_ref
                theta_ref = zeros(1,length(x_ref));
                for i = 1: length(x_ref)-1 
                    theta_ref(i) = atan2( (y_ref(i+1) - y_ref(i)), (x_ref(i+1) - x_ref(i)));
                end
                % Setting last element to previous angle.
                theta_ref(length(x_ref)) = theta_ref(length(x_ref)-1);
                theta_ref = theta_ref - theta_ref(1);
            end
            
            % Correcting x/y on path
            [pose_ref,x_ref,y_ref,doors]=path_door_correction(d_i,pose_ref,x_ref,y_ref,doors,error);
            
            
            if (d_i == 2 || d_i == 5 || d_i == 7 || d_i == 10 || d_i == 16)
                trajectory_plot = figure(2);
                gg = plot(x_ref,y_ref,'o',x_ref,y_ref,'-',doors_rotated(1,:)/1000,doors_rotated(2,:)/1000,'*','LineWidth',2);
            end
        end
        
        
        
        
        
        
        % WHAT IS THIS SPECIAL CASE AND WHAT CAN WE DO WITH IT?        
        % Special case for corner doors facing striaght forward
        if norm([5.02, 18.36] - pos(1:2)) < 0.2 || norm([18.74, 5.13] - pos(1:2)) < 0.2
           pioneer_set_controls(sp, 0, 0);
           pause(3);
           
        end
        
        
%         figure(2);
%         hold on;
%         plot(pose_obs(k+1,1), pose_obs(k+1,2), 'm.');
%         drawnow;
%         hold off;
        
        k=k+1;
        %disp(['iteration',num2str(k)])
        
        waitfor(r);
    end
end

% figure(2)
% plot(pose_obs(:,1), pose_obs(:,2), 'g.')

pioneer_set_controls(sp, 0, 0);
pioneer_close(sp);
fclose(lidar);
stats = statistics(r)

function data = loop(sp, pose_ref)
    
    % TUNING
    K1 = 0.5; % Artikkel: 0.41 2.94 1.42 0.5
    K2 = 2.5;%2.3;
    K3 = 1.5; %1.5;
    v_max = 0.85;
    
    
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
