function [error,doors]=door_turn(doors,d_i,sp,lidar,distance_to_wall,side)
    
    pioneer_set_controls(sp, 0, 0);
    pause(1);
    
    % Independent for each side
    if side == -1 %Right side
        turn=-85;
    elseif side== 1
        turn=85;
    end
    
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
    if abs(error) > 0.05
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
    
    
    if d_i == 14
        % turn back
        pioneer_set_controls(sp, 0, -turn);
        pause(1);
        pioneer_set_controls(sp, 0, 0);
        pause(1);
        pioneer_set_controls(sp, 0, -turn);
        pause(1);
        pioneer_set_controls(sp, 0, 0);
        pause(1);
        
        scan = LidarScan(lidar);
        % Check the door state:
        door_state=Doors(scan,distance_to_wall);
        pioneer_set_controls(sp, 0, turn);
        pause(1);
        pioneer_set_controls(sp, 0, 0);
        pause(1);
        
        %backward
        pioneer_set_controls(sp, -300, 0);
        pause(1.433333);
        pioneer_set_controls(sp, 0, 0);
        pause(1);
    else
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
end
