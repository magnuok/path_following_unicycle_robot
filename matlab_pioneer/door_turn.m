function [distance_to_wall, error]=door_turn(sp,lidar,scan,side)
    
    pioneer_set_controls(sp, 0, 0);
    pause(1);
    
    % Independent for each side
    if side == -1 %Right side
        turn=-85;
        scan_aux=scan(40:125);
    elseif side== 1
        turn=85;
        scan_aux=scan(547:627);
    end
    
    %Reading from the scan and finding the distance to wall
    for l=1:1:length(scan_aux)
        if scan_aux(l) < 10
            scan_aux(l)=5000;
        end
    end

    %             last_odom_door = odom_door;
    %             odom_door = pose_obs(1:2);
    %             last_distance_to_wall = distance_to_wall;

    distance_to_wall = min(scan_aux)/1000;
    
    % Go forward
    pioneer_set_controls(sp, 300, 0);
    pause(1.433333);
    pioneer_set_controls(sp, 0, 0);   
    pause(1);
    %Go turn
    pioneer_set_controls(sp, 0, turn);
    pause(1);
    pioneer_set_controls(sp, 0, 0);
    pause(1);

    % Check the distance to the wall

    scan = LidarScan(lidar);
    % Check the door state:
    door_state=Doors(scan,distance_to_wall);
    error = distance_to_wall - doors(d_i, 5);

    % Robot position correction, knowing the error, moves forward or
    % backward
    if abs(error) >0.15
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
    
    % turn back
    pioneer_set_controls(sp, 0, -turn);
    pause(1);
    pioneer_set_controls(sp, 0, 0);
    pause(1);
    %backward
    pioneer_set_controls(sp, -300, 0);
    pause(1.433333);
    pioneer_set_controls(sp, 0, 0);
    pause(1);

          
    end

        
%         last_odom_door = odom_door;
%         odom_door = data(1:2);
%         last_distance_to_wall = distance_to_wall;
% 
%         % Correct path with measured error
%         error = distance_to_wall - doors(d_i, 5);
%  
% end
