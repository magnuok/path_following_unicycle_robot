%% Create map
clear all;
close all;
clf; 

% TUNING VARIABLES
 radius = 0.2;
 measurment_points = 15;
 
% INTERPOLATION AND PLOTS

path = [-0.5, 0; -1, 0; -2, 0];
%path = dlmread('path_nice_corrected_2.txt');

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

% Start in (0,0)
x = x - x(1);
y = y - y(1);
x_ref = x_ref - x_ref(1);
y_ref = y_ref - y_ref(1);

% Rotate points
%theta = atan2( (y_ref(2) - y_ref(1)) , (x_ref(2) - x_ref(1) ));
%R = [cos(-theta) -sin(-theta); sin(-theta) cos(-theta)];
%trajectory_rotated = R*[x_ref ; y_ref];

% Rotate interploation points
%interp_roateted = R*[x ; y];
% pick out the vectors of rotated x- and y-data
%x_ref = trajectory_rotated(1,:);
%y_ref = trajectory_rotated(2,:);
%x = interp_roateted(1,:);
%y = interp_roateted(2,:);

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
gg = plot(x_ref,y_ref,'o',x_ref,y_ref,'-','LineWidth',2);
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
for i = 1:length(x_ref)
    %// center
    c = [x_ref(i) y_ref(i)];

    pos = [c-radius 2*radius 2*radius];
    rectangle('Position',pos,'Curvature',[1 1])
    axis equal
    
    text(x_ref(i) + 0.1,y_ref(i) + 0.1 ,num2str(i),'Color','k')
end
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

% iteration counter
k = 1;

% In Hz
r = robotics.Rate(20);

sp = serial_port_start();
%CONFIG: timer_period = 0.1. Can change to lower maybe?
pioneer_init(sp);

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