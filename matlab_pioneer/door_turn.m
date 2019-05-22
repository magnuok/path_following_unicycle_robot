function [error,doors]=door_turn(doors,d_i,sp,lidar,distance_to_wall,side)
    
    pioneer_set_controls(sp, 0, 0);
    pause(1);
    
    % Independent for each side
    if side == -1 %Right side
        turn=-85;
    elseif side== 1
        turn=85;
    end
    


    %             last_odom_door = odom_door;
    %             odom_door = pose_obs(1:2);
    %             last_distance_to_wall = distance_to_wall;

    
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
