function [pose_ref,x_ref,y_ref,doors]=path_door_correction(pose_ref,x_ref,y_ref,doors,error)
    
    if abs(error) >0.15 
        if (doors(d_i, 6) == 0)                
            % add in x-direction
            if (doors(d_i, 7) == 1)
                x_ref;
                error;
                x_ref = x_ref + error; % ADD IN MILLI!!
                pose_ref(1) = pose_ref(1) + error;
                x_ref;
                doors(:,1) = doors(:,1) + 1000*error;
            % subtract in x-direction
            else
                x_ref;
                error;
                x_ref = x_ref - error;
                pose_ref(1) = pose_ref(1) - error;
                x_ref;
                doors(:,1) = doors(:,1) - 1000*error;
            end
        % y-direction
        else
            % add in y-direction
            if (doors(d_i, 7) == 1)
                y_ref;
                error;
                pose_ref(2) = pose_ref(2) + error;
                y_ref = y_ref + error;
                doors(:,2) = doors(:,2) + 1000*error;

            % subtract in y-direction
            else
                y_ref;
                error;
                pose_ref(2) = pose_ref(2) - error;
                y_ref = y_ref - error;
                doors(:,2) = doors(:,2) - 1000*error;

            end
        end
    end
end