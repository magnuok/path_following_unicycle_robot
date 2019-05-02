function path = create_path()
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

end