%% Create map
clear all;
close all;
clf
h = 0.1;

% Create a path and plot

path = [0.00    0.00;
        1.00    1.00;
        5.00    8.00;
        7.00    8.00;
        11.00   10.00;
        15.00   15.00;
        17.00   20.00];


image = imread('map.pgm');
% Crop image to relevant area
imageCropped = image(1:1100,1:1300);

image = imageCropped < 100;

% create occupancy grid. Free space is 0, for occupied space is 1
% .yaml: Resolution of the map, meters / pixel = 0.020000
% . BinaryOccupancyGrid: resoultion of cells per meter

map = robotics.BinaryOccupancyGrid(image, 50);

% Copy and inflate the map to factor in the robot's size for obstacle 
% avoidance
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
 startLocation = [9 14.5];
 endLocation = [24.4 5.2];
 % SW corner [24.4 5.2]

 % Search for a solution between start and end location.
 % Continue to add nodes until a path is found.
 
 % path matrix containing [x,y] points
 path = findpath(prm, startLocation, endLocation);
 
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

i = 1;
while i < length(path)-1
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

% Set number of points in reference trajectory
measurment_points = 30;

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

figure(2)
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
% figure(3)
% axis([0,x_ref(end),0,pi])
% gg = plot(x_ref, theta_ref, '-','LineWidth',2);
% title('THETA')
% hl=legend('$\theta_{ref}$', 'AutoUpdate','off');
% set(hl,'Interpreter','latex')
% set(gg,"LineWidth",1.5)
% gg=xlabel("x - [m]");
% set(gg,"Fontsize",14);
% gg=ylabel("rad");
% set(gg,"Fontsize",14);
% hold on;

%% POSITION TRACKING

% start pos = [x, y, theta]
% HVORFOR + pi her på theta_ref
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
        K3 = 1;
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
        
        
         % plotting simulated trajectory
         figure(2)
         plot(pose(k+1,1), pose(k+1,2), 'k.')
         drawnow
%         
%         % plotting simulated theta
%         figure(3)
%         plot(pose(k+1,1), pose(k+1,3), 'go')
%         drawnow
        
        k=k+1;
        disp(['iteration',num2str(k)])

    end
end

%figure(2)
%plot(pose(:,1), pose(:,2), 'k-')
%figure(3)
%plot(pose(:,1), pose(:,3), 'k-')
