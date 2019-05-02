function path = path_planner()
 % PRM creates a roadmap path planner object for the environment map 
 % specified in the Map property.
 prm = robotics.PRM;
 prm.Map = mapInflated;
 prm.NumNodes = 2500;
 % ConnectionDistance is an upper threshold for points that are connected 
 % in the roadmap
 
 prm.ConnectionDistance = 3;
  
 % Lab into hallway. NEW MAP 
 % startLocation = [2.59 20.71; 2.59 17.15; 4.37 16.07; 7.4 15];
 % endLocation = [2.59 17.15; 4.37 16.07; 7.4 15; 7.01 12.41];
 
 % HALLWAY. NEW MAP
 % startLocation =  [7.01 12.41; 6.91 2.53; 7.59 1.91; 18.77 2.07; 19.43 2.55; 19.21 15.41; 18.49 16.05];
 % endLocation = [6.91 2.53; 7.59 1.91; 18.77 2.07; 19.43 2.55; 19.21 15.41; 18.49 16.05; 7.19 15.89];
  
 % HALLWAY into LAB. NEW MAP
 % startLocation =  [7.19 15.89; 3.811 16.22];
 % endLocation = [3.811 16.22; 2.59 20.71];
 
 % WHOLE PATH. NEW MAP
  start_coordinates = [2.59 20.71];
  startLocation = [2.59 20.71; 2.59 17.15; 4.37 16.07; 7.4 15; 7.01 12.41; 6.91 2.53; 7.59 1.91; 18.77 2.07; 19.43 2.55; 19.21 15.41; 18.49 16.05; 7.19 15.89; 3.811 16.22];
  endLocation = [2.59 17.15; 4.37 16.07; 7.4 15; 7.01 12.41; 6.91 2.53; 7.59 1.91; 18.77 2.07; 19.43 2.55; 19.21 15.41; 18.49 16.05; 7.19 15.89; 3.811 16.22; 2.59 20.71];
  measurment_points = 200;
  
  
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
        prm.NumNodes = prm.NumNodes + 500;
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

end