function door_state = Doors(scan,distance_to_wall)
% Detects the state of the door by using 80 frontal laser readings.
% Calculates the average and classificates based on tests made.
    [signalclose, Fs]=audioread('door_closed.mp3');
    [signalopen, Fs]=audioread('opened_door.mp3');
    [signalhalfopen, Fs]=audioread('door_half_opened.mp3');

    %distance_to_wall=distance_to_wall/1000;
    dista=0;

    b=0;
     for l=331:1:351
         b=b+1;
         if scan(l:l) < 10
             dista=dista+4000;
             door(b)=4000;
         else
              dista=dista+scan(l:l);
              door(b)=scan(l:l);
         end
       
    end
    dista=dista/(1000*b)
    dist2=median(door)
    
    %If needed, change 0.6 for distance_to_wall

    %if dista < (distance_to_wall + 0.1)
    %distance_to_wall=0.7040;
    %dista=0.6250;
    if dista < distance_to_wall + 0.15

        door_state=-1;
        soundsc(signalclose,Fs);
        pause(2);
        %If needed, change 1 for distance_to_wall +0.5

        elseif dista > (distance_to_wall + 0.4)

            door_state=1;
            soundsc(signalopen,Fs);
            pause(2);
        else
            door_state=0;
            soundsc(signalhalfopen,Fs);
            pause(2);
    end

    
end
