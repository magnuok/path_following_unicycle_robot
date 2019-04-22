%% Create map
clear all;
close all;
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
robotRadius = 0.3; % TODO. Check dimention
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
 prm.ConnectionDistance = 2;
 
 % From HALLWAY to ELEVATOR:
 startLocation = [8.8 14.6];
 endLocation = [15 12];
 
 % From LAB to ELEVATOR:
 % startLocation = [4 16 ; 8.8 14.6];
 % endLocation = [8.8 14.6 ; 15 12];
 
 % WHOLE PATH
 % startLocation = [4 16; 8.8 14.6; 12.5 1.4; 24.4 5; 20.5 17.4; 9 14.5];
 % endLocation = [8.8 14.6; 12.5 1.4; 24.4 5; 20.5 17.4; 9 14.5; 5 17];

 % Search for a solution between start and end location.
 % Continue to add nodes until a path is found.
 
 path = [];
 
 for i = 1:size(startLocation, 1)
     % path matrix containing [x,y] points
     sub_path = findpath(prm, startLocation(i,:), endLocation(i,:));

     % if path is not found, add more nodes
    iterations = 1;
    while isempty(sub_path)
        % Can tune this to add more each round
        prm.NumNodes = prm.NumNodes + 100;
        update(prm);
        sub_path = findpath(prm, startLocation(i,:), endLocation(i,:));
        fprintf('Iteration: %i\n', iterations);
        iterations = iterations+1;
    end
    
    path = [path ; sub_path];
    show(prm)
    hold on;
 end
 
disp('Found path');

show(prm)
hold on;

% Removing extraneous nodes to be interpolated
j = 1;
while j < length(path)
    if(norm( path(j,1:2) - path(j+1,1:2) ) > 1)
        j = j+1;
    else
        path(j+1,:) = [];
        j = 1;
    end

end

%% INTERPOLATION AND PLOTS
%close all;
% Set number of points in reference trajectory
measurment_points = 20;

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

% Plotting reference trajectory
figure(1);
plot(x,y,'o',x_ref,y_ref,'-','LineWidth',2);

% Start in (0,0)
x = x - x(1);
y = y - y(1);
x_ref = x_ref - x_ref(1);
y_ref = y_ref - y_ref(1);

% Rotate points
theta = atan( (y_ref(2) - y_ref(1)) / (x_ref(2) - x_ref(1) ));
R = [cos(-theta) -sin(-theta); sin(-theta) cos(-theta)];
trajectory_rotated = R*[x_ref ; y_ref];
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
    theta_ref(i) = atan( (y_ref(i+1) - y_ref(i)) / (x_ref(i+1) - x_ref(i) ));
end
theta_ref = theta_ref - theta_ref(1);

trajectory_plot = figure(2);
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

% only printing theta for tests. No meaning for whole path.
if (size(startLocation, 1) == 1)
    % Plotting theta_ref
    theta_plot = figure(3);
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
end

%% POSITION TRACKING

% start pos = [x, y, theta]
pose(1,:) = [x_ref(1), y_ref(1), theta_ref(1)];

% observed position
pose_obs(1,:) = pose(1,:);

theta_obs(1) = pose_obs(1,3);

% calculated speeds
dot_pose(1,:) = [0,0,0];

% plot start position (x,y)
figure(2)
plot(pose(1,1),pose(1,2),'ro');


% iteration counter
k=1;

for k1 = 1:length(x_ref)
    
    % Changing reference
    pose_ref = [x_ref(k1),y_ref(k1)];
    
    while norm(pose_obs(k,1:2) - pose_ref) > 0.3
        
        % Calculating errors and variables
        e(k)=norm(pose_ref - pose_obs(k,1:2));
        phi(k)=atan2(pose_ref(2)-pose_obs(k,2),pose_ref(1)-pose_obs(k,1));
        alpha(k) = phi(k) - pose_obs(k,3);
        
        % Compensating for if angle is more than pi or less than pi
        if alpha(k)>pi
            alpha(k) = alpha(k)-2*pi;
        elseif alpha(k) < -pi
            alpha(k) = alpha(k) + 2*pi;
        end
        
        % Tuning variables
        K1 = 1;
        K2 = 1;
        K3 = 3;
        v_max=1;
        
        % Control law
        % Put in a wind-up parameter here and tune variables?
        
        v(k) = v_max*tanh(K1*e(k));
        w(k) = v_max*((1+K2*phi(k))*tanh(K1*e(k))/e(k)*sin(alpha(k))+K3*tanh(alpha(k)));
        
        % Change i x,y,theta position
        pose(k+1,1) = pose(k,1) + h*cos(pose(k,3))*v(k);
        pose(k+1,2) = pose(k,2) + h*sin(pose(k,3))*v(k);
        pose(k+1,3) = pose(k,3) + h*w(k);
        
        if(pose(k+1,3)>pi)
            pose(k+1,3)=pose(k+1,3)-2*pi;
        elseif (pose(k+1,3)<-pi)
            pose(k+1,3)=pose(k+1,3)+2*pi;
        end
        
        % Setting pose observed to last post
        pose_obs(k+1,:) = pose(k+1,:);
        
        
%          % plotting simulated trajectory
%          figure(2)
%          plot(pose(k+1,1), pose(k+1,2), 'k.')
%          drawnow
%         
%         % plotting simulated theta
%         figure(3)
%         plot(pose(k+1,1), pose(k+1,3), 'go')
%         drawnow
        
        k=k+1;
        disp(['iteration',num2str(k)])

    end
end

figure(2)
plot(pose(:,1), pose(:,2), 'k.')

% only printing theta for tests. No meaning for whole path.
if (length(startLocation) == 2)
    figure(3)
    plot(pose(:,1), pose(:,3), 'k.')
end