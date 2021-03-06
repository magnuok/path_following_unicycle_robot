figure(5);
image_2 = imread('floor_plant_1.jpg');
% Crop image to relevant area
imageCropped_2 = image_2(1:1150,1:1100);

image_2 = imageCropped_2 < 100;

% create occupancy grid. Free space is 0, for occupied space is 1
% .yaml: Resolution of the map, meters / pixel = 0.020000
% . BinaryOccupancyGrid: resoultion of cells per meter

map_2 = robotics.BinaryOccupancyGrid(image_2, 50);

show(map_2)
hold on;
plot(2.6,20.82, '*')
% Copy and inflate the map to factor in the robot's size for obstacle 
% avoidance. Setting higher to get trajectory in middle of halway.
robotRadius = 0; % TODO. Check dimention
mapInflated_2 = copy(map);
inflate(mapInflated_2, robotRadius);

%show(mapInflated_2)
hold on;

plot(2.59,20.71, '*')