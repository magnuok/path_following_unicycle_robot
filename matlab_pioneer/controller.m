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
robotRadius = 0.15; %0.2; %0.1
mapInflated = copy(map);
inflate(mapInflated, robotRadius);
show(mapInflated)
hold on;

global doors;
doors = dlmread('doors.txt'); % [x,y,bol] bol=1 right bol=0 left

 % TUNING VARIABLES
radius = 0.3; %0.25;
measurment_points = 250;

%path = path_planner();
 
% INTERPOLATION AND PLOTS
%path = dlmread('path_nice_verynice_test.txt');
path = dlmread('path_nice_corrected_2.txt');

x = path(:,1)';
y = path(:,2)';

%corr_points = [21.53 8.4; 21.53 6.4; 21.53 6.4; 21.53 6.4];

corr_points = [7,15.83;
8.38,15.83;
17.3,15.83;
17.65,15.83;
21.53,8.08;
21.53,6.54;
20.26,1.5;
19.6,1.5;
12.42,1.5;
11.36, 1.5;
6.93,3.02;
6.93,4.5;
6.93,7.26;
6.93,8.9;
6.93,13.7;
6.93,14.5;
0.0,0.0;
0.0,0.0];


door_front = [20.9, 15.85; 7.45, 2.0];

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
plot(corr_points(:,1), corr_points(:,2), 'm+');

% Doors
doors_x = doors(:,1) - x(1)*1000;
doors_y = doors(:,2) - y(1)*1000;

% Correction points
corr_points_x = corr_points(:,1) - x(1);
corr_points_y = corr_points(:,2) - y(1);


door_front_x = door_front(:,1) - x(1);
door_front_y = door_front(:,2) - y(1);


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
door_front_rotated = R*[door_front_x' ; door_front_y'];
door_front = door_front_rotated';

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
axis([map.XWorldLimits(1),map.XWorldLimits(2),map.YWorldLimits(1),map.YWorldLimits(2)])
gg = plot(x_ref,y_ref,'o',x_ref,y_ref,'-',doors_rotated(1,:)/1000,doors_rotated(2,:)/1000,'*',corr_points(:,1),corr_points(:,2),'m+','LineWidth',2);
title('TRAJECTORY')
hl=legend('$Interpolation points (x,y)$' ,'$(x_{ref},y_{ref})$','$Door coordinates (x,y)$','$Correctionpoints$', 'AutoUpdate','off');
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
a=0;
a_1=1;
a_2=2;
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
in_range = false;



for k1 = 1:length(x_ref)
    
    % Changing reference
    
    pose_ref = [x_ref(k1),y_ref(k1), theta_ref(k1)];
    
    while norm(pose_obs(k,1:2) - pose_ref(1:2)) > radius
        
        % this will be performed every dadada seconds
        % data = [pose_new, e, phi, alpha, v, w]
        
        % drive slowly forwrd
        if in_range == true
            data = loop(sp, pose_ref,1);
        else % as usual
            data = loop(sp, pose_ref,0);
        end
        
        pose_obs(k+1,:) = data(1:3);

        % Find close by doors
        pos = data(1:2)*1000;
        range_threshold = 1100; % Search for the door inside threshold
        nearby_door_right = [];
        nearby_door_left = [];
        door_detected = [0 0];
        in_range = false;
        
        for i = 1:length(doors(:,1))
            range = norm([doors(i,1),doors(i,2)] - pos(1:2) );
            
            % if it is close enough and not discovered.
            if range < range_threshold && doors(i,4) == 0 
                if doors(i,3) == 1
                    nearby_door_right = [doors(i,:), i];% adding index because needed to change detected or not parameter to true/false
                    d_i = i;
                    in_range = true;
                else
                    nearby_door_left = [doors(i,:), i]; 
                    d_i = i;
                    in_range = true;
                end
            end
        end

        
        %if close to door, search for them
        if ~isempty(nearby_door_right) || ~isempty(nearby_door_left)
            scan = LidarScan(lidar);
            door_detected = door_detector(nearby_door_right, nearby_door_left, scan, d_i);
            
        end
        
        % look at this threshold. Could be bigger
        % a_1 and a_2 is used for the index of each position. This way,
        % only on distance is calculated on each side and ordered and shit.
        if (norm(data(1:2) - corr_points(a_1,:)) < 0.3) && (a_1 < a_2)
            scan = LidarScan(lidar);
            side=1;
            [distance_to_wall_1]=distance_calc(scan,side);
            odom_1=data(1:2);
             a_1=a_1+2
        elseif (norm(data(1:2) - corr_points(a_2,:)) < 0.3) && (a_2 < a_1)
            a_2=a_2+2
            %test
            pioneer_set_controls(sp, 0, 0);
            pause(0.3);
            %test
            scan = LidarScan(lidar);
            side=1;
            [distance_to_wall_2]=distance_calc(scan,side);
            odom_2=data(1:2);
            distance = norm(odom_1 - odom_2);
            delta = distance_to_wall_2 - distance_to_wall_1;
            
            theta_error = -atan(delta/distance);
            % Doors
            doors_x = doors(:,1) - 1000*odom_1(1);
            doors_y = doors(:,2) - 1000*odom_1(2);
            corr_points_x = corr_points(:,1) - odom_1(1);
            corr_points_y = corr_points(:,2) - odom_1(2);
            door_front_x = door_front(:,1) - odom_1(1);
            door_front_y = door_front(:,2) - odom_1(2);

            % Rotate points
            R = [cos(-theta_error) -sin(-theta_error); sin(-theta_error) cos(-theta_error)];
            trajectory_rotated = R*[x_ref - odom_1(1) ; y_ref - odom_1(2)];
            doors_rotated = R*[doors_x' ; doors_y'];
            corr_points_rotated = R*[corr_points_x' ; corr_points_y'];
            door_front_rotated = R*[door_front_x' ; door_front_y'];
            
            % put back
            doors_rotated(1,:) = doors_rotated(1,:) + odom_1(1)*1000;
            doors_rotated(2,:) = doors_rotated(2,:) + odom_1(2)*1000;
            doors(:,1:2) = doors_rotated';
            
            corr_points_rotated(1,:) = corr_points_rotated(1,:) + odom_1(1);
            corr_points_rotated(2,:) = corr_points_rotated(2,:) + odom_1(2);
            corr_points = corr_points_rotated';
            
            door_front_rotated(1,:) = door_front_rotated(1,:) + odom_1(1);
            door_front_rotated(2,:) = door_front_rotated(2,:) + odom_1(2);
            door_front = door_front_rotated';
            
            
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
            
            % DELETING CORRECTION POINTS
            %corr_points(1,:) = [];
            %corr_points(2,:) = [];
            
            % Plotting
%             trajectory_plot = figure(2);
%             gg = plot(x_ref,y_ref,'-',doors_rotated(1,:)/1000,doors_rotated(2,:)/1000,'*',corr_points(:,1),corr_points(:,2),'m+','LineWidth',2);
%             hold on;
        end
    
        
        % door is detected, drive to evaluate if door is open or not.
        if door_detected(1) == 1 % Door on the right side 
             side=-1;
            [distance_to_wall]=distance_calc(scan,side);
            [error,doors]=door_turn(doors,d_i,sp,lidar,distance_to_wall,side);
            [distance_to_wall]=distance_calc(scan,side);
            [pose_ref,x_ref,y_ref,doors,corr_points, door_front]=path_door_correction(d_i,pose_ref,x_ref,y_ref,doors,error,corr_points,door_front);

        
        elseif door_detected(2) == 1 %Door on the left side
            side=1;
            last_odom_door = odom_door;
            odom_door = data(1:2);
            last_distance_to_wall = distance_to_wall;
            distance_to_wall = distance_calc(scan, side);

            [error,doors]=door_turn(doors, d_i, sp, lidar, distance_to_wall, side);
            [distance_to_wall]=distance_calc(scan, side);
            [pose_ref,x_ref,y_ref,doors, corr_points, door_front] = path_door_correction(d_i,pose_ref, x_ref, y_ref, doors, error, corr_points, door_front);
        end
        
        
        if (norm(door_front(1,:) - data(1:2)) < 0.3 || norm(door_front(2,:) - data(1:2)) < 0.3 ) && a==0
             pioneer_set_controls(sp, 0, 0);
             pause(2);
%             pioneer_set_controls(sp, 300, 0);
%             pause(1.433333);
%             pioneer_set_controls(sp, 0, 0);   
%             pause(2);
%             soundsc(signalclose,Fs);
%             pause(2);
%             pioneer_set_controls(sp, -300, 0);
%             pause(1.433333);
%             pioneer_set_controls(sp, 0, 0);
%            pause(1);
            
            % Copy paste if works
            scan = LidarScan(lidar);
            scan_aux=scan(331:351);
            for l=1:1:length(scan_aux)
                if scan_aux(l) < 10
                    scan_aux(l)=5000;
                end
            end
            distance_to_wall = min(scan_aux)/1000
            % measure here
            error = distance_to_wall - 1.3 

            % Robot position correction, knowing the error, moves forward or
            % backward
            if abs(error) > 0.05
                if error > 0 
                    speeder=100;
                    time_error=((error*1000)/100)-0.4; 
                else
                    speeder=-100;
                    time_error=((-error*1000)/100)-0.4;
                end
                pause(1);
                pioneer_set_controls(sp, speeder, 0);
                pause(time_error);
                pioneer_set_controls(sp, 0, 0);           
                pause(3);
            end
            
            [pose_ref,x_ref,y_ref,doors, corr_points, door_front] = path_door_correction(7,pose_ref, x_ref, y_ref, doors, error, corr_points, door_front);
            a=1;
            
            trajectory_plot = figure(2);
            gg = plot(x_ref,y_ref,'-',doors_rotated(1,:)/1000,doors_rotated(2,:)/1000,'*',corr_points(:,1),corr_points(:,2),'m+','LineWidth',2);
            hold on;
            
            pause(2);
            soundsc(signalclose,Fs);
            pause(2);
            
        end
        
         if (norm(door_front(2,:) - data(1:2)) < 0.3 ) && a==1
%             pioneer_set_controls(sp, 0, 0);
%             pause(2);
%             pioneer_set_controls(sp, 300, 0);
%             pause(1.433333);
%             pioneer_set_controls(sp, 0, 0);
%             pause(1);
%             soundsc(signalclose,Fs);
%             pause(2);
%             pioneer_set_controls(sp, -300, 0);
%             pause(1.433333);
%             pioneer_set_controls(sp, 0, 0);
%             pause(1);

            scan = LidarScan(lidar);
            scan_aux=scan(331:351);
            for l=1:1:length(scan_aux)
                if scan_aux(l) < 10
                    scan_aux(l)=5000;
                end
            end
            distance_to_wall = min(scan_aux)/1000
            % measure here
            error = distance_to_wall - 1.3 

            % Robot position correction, knowing the error, moves forward or
            % backward
            if abs(error) > 0.05
                if error > 0 
                    speeder=100;
                    time_error=((error*1000)/100)-0.4; 
                else
                    speeder=-100;
                    time_error=((-error*1000)/100)-0.4;
                end
                pause(1);
                pioneer_set_controls(sp, speeder, 0);
                pause(time_error);
                pioneer_set_controls(sp, 0, 0);           
                pause(3);
            end
            
            [pose_ref,x_ref,y_ref,doors, corr_points, door_front] = path_door_correction(16,pose_ref, x_ref, y_ref, doors, error, corr_points, door_front);
            a=2;
            
            trajectory_plot = figure(2);
            gg = plot(x_ref,y_ref,'-',doors_rotated(1,:)/1000,doors_rotated(2,:)/1000,'*',corr_points(:,1),corr_points(:,2),'m+','LineWidth',2);
            hold on;
            
             pause(2);
             soundsc(signalclose,Fs);
             pause(2);
            
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
soundsc(signalshutdown,Fs1);
pioneer_set_controls(sp, 0, 0);
pioneer_close(sp);
fclose(lidar);
stats = statistics(r)

function data = loop(sp, pose_ref, flag)
    
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
    if flag == 1
        pioneer_set_controls(sp, 100, 0)
    else % if we are close to door
        pioneer_set_controls(sp, round(v*1000), round(w*(180/pi)))
    end
        
        
    % ROBOT
    pose_new = pose_obs;

    if(pose_new(3)>pi)
        pose_new(3)=pose_new(3)-2*pi;
    elseif (pose_new(3)<-pi)
        pose_new(3)=pose_new(3)+2*pi;
    end
    
    data = [pose_new, e, theta, alpha, v, w];
    
end
