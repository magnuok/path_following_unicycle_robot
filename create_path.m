%% Create map
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
robotRadius = 0.2; % TODO. Check dimention
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
 
 startLocation = [12 11];
 endLocation = [24.4 5.2];
 % SW corner [24.4 5.2]

 % Search for a solution between start and end location.
 % Continue to add nodes until a path is found.
 
 % path matrix containing [x,y] points
 path = findpath(prm, startLocation, endLocation);
 
 i = 1;
while isempty(path)
    % Can tune this to add more each round
    prm.NumNodes = prm.NumNodes + 100;
    update(prm);
    path = findpath(prm, startLocation, endLocation);
    fprintf('Iteration: %i\n', i);
    i = i+1;
end
disp('Found path');

show(prm)
hold on;

%% Create path
x = path(:,1)';
y = path(:,2)';
p = pchip(x,y);

t = linspace(0,1000,length(x));
xq = linspace(0,1000,1000);
% returns a piecewise polynomial structure
ppx = pchip(t,x);
ppy = pchip(t,y);
% evaluates the piecewise polynomial pp at the query points xq
xx=ppval(ppx, xq);
yy=ppval(ppy, xq);

plot(x,y,'o',xx,yy,'-','LineWidth',2);

%% Robot controller, vizualizer

viz = Visualizer2D;
viz.mapName = 'mapInflated';
pose = [xx(1); yy(1); pi/4];
viz(pose)

% Add waypoints to the visualization. This requires setting the hasWaypointsproperty 
%to true, as well as specifying an additional input as rows of [x y] coordinates.

release(viz) % Needed if changing hasWaypoints property after visualizing
%waypoints = [0, 0; 5, 5; 1, 3];
waypoints = [x', y'];
viz.hasWaypoints = true;
viz(pose,waypoints);

%Move the robot randomly 10 times and update the visualization.
%NOTE: If you run this section in a plain code script, this section will be animated.

for idx = 1:10
    pose(1) = pose(1) + 10;
    pose(2) = pose(2) + 10;
    viz(pose,waypoints)
    pause(1)
end


