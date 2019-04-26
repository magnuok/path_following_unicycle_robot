function status = get_door_status()%ranges)

%SetupLidar(); MATLAB MÅ VÆRE 1,  port_name ='/dev/tty.usbmodem1421';

%% TO TEST WITH LIDAR:
%ranges=LidarScan(lidar);

%% TO TEST WITH RANGES FILES:
%halfopendoor.txt
%opendoor.txt
%closeddoor.txt
ranges = dlmread('closeddoor.txt');

%% Define thresholds
eliminate_x = 900; % only look at x=[-eliminate_x:eliminate_x]
eliminate_y = 100; % only look at y>eliminate_y

open_threshold = 1500;
closed_threshold = 900;
search_range_door = 199;


%% Process ranges
x= [];
y=[];
for n = 1:length(ranges)
    if ranges(n)> 20
        xn = cosd(-30+ (n-1)*(240/682)) *ranges(n);
        yn = sind(-30+ (n-1)*(240/682)) *ranges(n);
        x=[x; xn];
        y=[y; yn];
    end

end
points=[];

%% Eliminate irrelevant points:
for i =1:length(x)
    if x(i)>-eliminate_x && x(i)<eliminate_x
        if y(i)>eliminate_y
            points=[points;x(i),y(i)];
        end
    end
end

plot(points(:,1),points(:,2));
hold on
x=points(:,1);
y=points(:,2);

%% Find y values in before - middle - after
for i=1:length(x)
    if x(i)<-search_range_door
        before=y(i);
        plot(x(i),before,'o');
        hold on
        break
    end
end
for i=1:length(x)
    if x(i)<search_range_door 
        after=y(i);
        plot(x(i),after,'o');
        hold on
        break
    end
end
for i=1:length(x)
    if x(i)<0 
        middle=y(i);
        plot(x(i),middle,'*');
        hold off
        break
    end
end

%% Check y values in before - middle - after
if before >open_threshold && middle >open_threshold && after >open_threshold
    status = 'open';
elseif before <closed_threshold && middle <closed_threshold && after <closed_threshold
    % check if closed or half open
    if sqrt((before-after)^2)<50 && sqrt((middle-after)^2)<50 && sqrt((before-middle)^2)<50 % threshold for difference between points in closed door
    status = 'closed';
    end
else
    status = 'halfopen';
end

%'open'
%'half'
%'closed'
end