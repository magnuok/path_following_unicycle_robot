%% Create map
clear all;
%close all;
clf
h = 0.1;

image = imread('map.pgm');
% Crop image to relevant area
imageCropped = image(1:1100,1:1300);

image = imageCropped < 100;

% create occupancy grid. Free space is 0, for occupied space is 1
% .yaml: Resolution of the map, meters / pixel = 0.020000
% . BinaryOccupancyGrid: resoultion of cells per meter

map = robotics.BinaryOccupancyGrid(image, 50);

% Copy and inflate the map to factor in the robot's size for obstacle 
% avoidance. Setting higher to get trajectory in middle of halway.
robotRadius = 0.4; % TODO. Check dimention
mapInflated = copy(map);
inflate(mapInflated, robotRadius);

show(mapInflated)

 % PRM creates a roadmap path planner object for the environment map 
 % specified in the Map property.
 prm = robotics.PRM;
 prm.Map = mapInflated;
 prm.NumNodes = 2000;
 % ConnectionDistance is an upper threshold for points that are connected 
 % in the roadmap
 prm.ConnectionDistance = 3;
 
 % ELEVATOR [12 11];
 startLocation = [8.8 14.6];
 endLocation = [24.4 5.2];
%  elevator hall [15 12]
 % SW corner [24.4 5.2]
%  room_door[8.8 14.6]

 % Search for a solution between start and end location.
 % Continue to add nodes until a path is found.
 
 % path matrix containing [x,y] points
 path = findpath(prm, startLocation, endLocation);
 
 % if path is not found, add more nodes
iterations = 1;
while isempty(path)
    % Can tune this to add more each round
    prm.NumNodes = prm.NumNodes + 100;
    update(prm);
    path = findpath(prm, startLocation, endLocation);
    fprintf('Iteration: %i\n', iterations);
    iterations = iterations+1;
end
disp('Found path');

% Removing extraneous nodes to be interpolated
i = 1;
while i < length(path)
    if(norm( path(i,1:2) - path(i+1,1:2) ) > 1)
        i = i+1;
    else
        path(i+1,:) = [];
        i = 1;
    end
    
end

show(prm)
hold on;

%% INTERPOLATION AND PLOTS
%close all;
% Set number of points in reference trajectory
measurment_points = 60;

x = path(:,1)';
y = path(:,2)';
p = pchip(x,y);

t = linspace(0,1000,length(x));
xq = linspace(0,1000,measurment_points);
% returns a piecewise polynomial structure
ppx = pchip(t,x);
ppy = pchip(t,y);
% evaluates the piecewise polynomial pp at the query points xq
x_ref=ppval(ppx, xq);
y_ref=ppval(ppy, xq);

% Plotting reference trajectory
plot(x,y,'o',x_ref,y_ref,'-','LineWidth',2);

figure(5)
axis([map.XWorldLimits(1),map.XWorldLimits(2),map.YWorldLimits(1),map.YWorldLimits(2)])
gg = plot(x,y,'o',x_ref,y_ref,'-','LineWidth',2);
title('TRAJECTORY')
hl=legend('$Interpolation points (x,y)$' ,'$(x_{ref},y_{ref})$', 'AutoUpdate','off');
set(hl,'Interpreter','latex')
set(gg,"LineWidth",1.5)
gg=xlabel("x - [m]");
set(gg,"Fontsize",14);
gg=ylabel("y - [m]");
set(gg,"Fontsize",14);
hold on;

% Calculating theta_ref
theta_ref = zeros(1,length(x_ref));
for i = 1: length(x_ref)-1 
    theta_ref(i) = atan( (y_ref(i+1) - y_ref(i)) / (x_ref(i+1) - x_ref(i) ));
end

% Plotting theta_ref
figure(6)
axis([0,x_ref(end),0,pi])
gg = plot(x_ref, theta_ref, '-','LineWidth',2);
title('THETA')
hl=legend('$\theta_{ref}$', 'AutoUpdate','off');
set(hl,'Interpreter','latex')
set(gg,"LineWidth",1.5)
gg=xlabel("x - [m]");
set(gg,"Fontsize",14);
gg=ylabel("rad");
set(gg,"Fontsize",14);
hold on;

%% POSITION TRACKING

% start pos = [x, y, theta]
pose(1,:) = [x_ref(1), y_ref(1), theta_ref(1)];

% observed position
pose_obs(1,:) = pose(1,:);

theta_obs(1) = pose_obs(1,3);

% calculated speeds
dot_pose(1,:) = [0,0,0];

% plot start position (x,y)
figure(7)
plot(pose(1,1),pose(1,2),'ro');

% Data for plotting, if needed
e = [];
phi = [];
alpha = [];
v = [];
w = [];

% iteration counter
k = 1;

% In Hz
r = robotics.Rate(10);

% Serial port object = pioneer
% need to change a parameter inside here
sp = serial_port_start();
%CONFIG: timer_period = 0.1. Can change to lower maybe?
pioneer_init(sp);
pause(2);


% MUST SET FIRST ODOM DATA TO WHAT'S IN MAP. That is making the path start
% (x=0,y=0,theta=0)!
figure(5)
hold on;
image = imread('map_corrected.pgm');
imageCropped = image(1:1100,1:1300);
image = imageCropped < 100;
map = robotics.BinaryOccupancyGrid(image, 50);
show (map)
hold off;
i=0;
odemetry(1,:) = zeros(1,3)
for k1 = 1:length(x_ref)
    
    % Changing reference
    pose_ref = [x_ref(k1),y_ref(k1)];
    
    while norm(pose_obs(k,1:2) - pose_ref) > 0.3
        
        % this will be performed every dadada seconds
        % data = [pose_new, e, phi, alpha, v, w]
        i= i+1;
        data = loop(sp, pose_ref, pose_obs(k,:), i, pose_obs, odemetry(k,:));
        i
        pose_obs(k+1,:) = data(1:3);
        e(k) = data(4);
        phi(k) = data(5);
        alpha(k) = data(6);
        v(k) = data(7);
        w(k) = data(8);
        odemetry(k+1,:) = data(9:11)
         % plotting simulated trajectory
         figure(5)
         hold on;
         plot(pose_obs(k+1,1), pose_obs(k+1,2), 'k.')
         drawnow
         hold off;
        
        k=k+1;
        disp(['iteration',num2str(k)])
        waitfor(r);
    end
end

figure(3)
plot(pose(:,1), pose(:,2), 'k.')
figure(4)
plot(pose(:,1), pose(:,3), 'k.')
stats = statistics(r)


function data = loop(sp, pose_ref, pose_obs, i, start, odemetry)
    % pose_obs is uneccassary when testing on robot

    % READ ODOMETRY HERE to get pose_obs
%     pose_odm = pioneer_read_odometry(pose_obs(1,:), a);
%     odm = pioneer_read_odometry();
    if i==1 
    start(i,:)= pose_obs(1,:);
    end
    odeme = pioneer_read_odometry()
%     odemetry(1) = odeme(1);
%     odemetry(2) = odeme(2);
%     odemetry(3) = odeme(3); 
    if( odeme(3) <= 2048)
        pose_obs(3) = odeme(3) * (pi / 2048);
    else
        pose_obs(3) = -(odeme(3)-2048) * (pi / 2048);
    end
    
    if i > 1
       pose_obs(1) = start(i-1, 1) + ((odeme(1) - odemetry(1))/1000);
       pose_obs(2) = start(i-1, 2) + ((odeme(2) - odemetry(2))/1000) ;
%        if ()
%        pose_obs(3) = start(i-1, 3) + pose_obs(3) ;
%     odemetry(i-1,1)
%     (odeme(1) - odemetry(1))
    end
%     pos_corected = start(i,:);

pose_obs

%     pos_corected(i,:) = odm_correct(i, pose_obs)
%     pose_obs = start(i,:);
    %convert to meter from mm and robots angular
%     pose_obs(1) = pose_obs(1)/1000;
%     pose_obs(2) = pose_obs(2)/1000;
%     if( pose_obs(3) <= 2048)
%         pose_obs(3) = pose_obs(3) * (pi / 2048);
%     else
%         pose_obs(3) = -(pose_obs(3)-2048) * (pi / 2048);
%     end
%     %convert to meter from mm and robots angular
%     pose_odm(1) = pose_odm(1)/1000;
%     pose_odm(2) = pose_odm(2)/1000;
%     if( pose_odm(3) <= 2048)
%         pose_odm(3) = pose_odm(3) * (pi / 2048);
%     else
%         pose_odm(3) = -(pose_odm(3)-2048) * (pi / 2048);
%     end
%     
%     pose_obs = pose_odm;
    % Calculating errors and variables
    e = norm(pose_ref - pose_obs(1:2))
    phi = atan2(pose_ref(2)-pose_obs(2),pose_ref(1)-pose_obs(1));
    alpha = phi - pose_obs(3);

    % Compensating for if angle is more than pi or less than pi
    if alpha > pi
        alpha = alpha - 2*pi;
    elseif alpha < -pi
        alpha = alpha + 2*pi;
    end

    % Tuning variables
    h = 0.1;
    K1 = 1;
    K2 = 10;
    K3 = 0.1;
    v_max=0.5;

    % Control law
    % Put in a wind-up parameter here and tune variables?
    v = v_max*tanh(K1*e);
    w = v_max*((1+K2*phi)*tanh(K1*e)/e*sin(alpha)+K3*tanh(alpha));
    
    % SET v AND w here
     pioneer_set_controls(sp, round(v*1000), round(w*(180/pi)))
    
    % SIMULATION
    % Change i x,y,theta position
%     pose_new = zeros(1,3);
%     pose_new(1) = pose_obs(1) + h*cos(pose_obs(3))*v;
%     pose_new(2) = pose_obs(2) + h*sin(pose_obs(3))*v;
%     pose_new(3) = pose_obs(3) + h*w;

    % ROBOT
    pose_new = pose_obs;

    if(pose_new(3)>pi)
        pose_new(3)=pose_new(3)-2*pi;
    elseif (pose_new(3)<-pi)
        pose_new(3)=pose_new(3)+2*pi;
    end
    
    data = [pose_new, e, phi, alpha, v, w, odeme]
end

% function pos_corected = odm_correct(i, start_pos)
%     if i==1 
%     start(i,:)= start_pos
%     end
%     odeme = pioneer_read_odometry();
%     odemetry(i,1) = odeme(1);
%     odemetry(i,2) = odeme(2);
%     odemetry(i,3) = odeme(3); 
%     if( odeme(3) <= 2048)
%         odemetry(i,3) = odeme(3) * (pi / 2048);
%     else
%         odemetry(i,3) = -(odeme(3)-2048) * (pi / 2048);
%     end
%     
%     if i > 1
%        i
%        start(i, 1) = start(i-1, 1) + (odeme(1) - odemetry(i-1,1)) ;
%        start(i, 2) = start(i-1, 2) + (odeme(2) - odemetry(i-1,2)) ;
%        start(i, 3) = start(i-1, 3) + (odeme(3) - odemetry(i-1,3)) ;
%     end
%     pos_corected = start(i,:);
% end