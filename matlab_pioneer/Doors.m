clear
close all;
sp=serial_port_start('COM5')
pause(2);
pioneer_init(sp)
pause(2);
t=200;


[signalclose, Fs]=audioread('door_closed.mp3');
[signalopen, Fs]=audioread('opened_door.mp3');
[signalhalfopen, Fs]=audioread('door_half_opened.mp3');



% Use the laser:
% Door will correspond to a jump in the readings
% First derivative go crazy
% Use sin/cos/tg((distance to crazy first derivative)/distance to wall) 
% Correct the odometry this way








%pioneer_set_controls(sp, 220, 0);
 status =2; actual=1;
 pause(0.2);
 
 %Status = 0  Leaving the room;
 %Status=1 First corridor;
 i=0;
 son=zeros(200000,8);
 odemetry=zeros(200000,3);
 while status < 4
    i=i+1;
    sonar=pioneer_read_sonars();
    son(i,1)=sonar(1)/1000;son(i,2)=sonar(2)/1000;
    son(i,3)=sonar(3)/1000;son(i,4)=sonar(4)/1000;
    son(i,5)=sonar(5)/1000;son(i,6)=sonar(6)/1000;
    son(i,7)=sonar(7)/1000;son(i,8)=sonar(8)/1000;

    % CLOSED DOOR
    Distance_Wall=0.75;%Read by the laser when the door is found
        if ( (son(i,4)/son(i,5)) <1.2  ||(son(i,4)/son(i,5)) > 0.8)  && son(i,4)< Distance_Wall
            soundsc(signalclose,Fs);
            DoorClosed=1
            status = 5;
            %pause(30);
        end
    % OPEN DOOR
        if ( (son(i,4)/son(i,5)) <1.2  ||(son(i,4)/son(i,5)) > 0.8)  && son(i,4)> Distance_Wall +0.2
            soundsc(signalopen,Fs);
            DoorOpen=1
            status = 5;
        end
        if ( (son(i,4)/son(i,5)) >2.2)
            soundsc(signalhalfopen,Fs);
            LEFTDoorOpen=1
            status = 5;  
        end
        if ( (son(i,4)/son(i,5)) <0.8) 
            soundsc(signalhalfopen,Fs);
            RIGHTDoorOpen=1
            status = 5;
        end
    %     if son(i,4)< Distance_Wall +0.2 && son(i,5)> Distance_Wall + 0.5
    %          %soundsc(signalopen,Fs);
    %          LEFTDoorOpen=1
    %      end
    %     if son(i,4)> Distance_Wall + 0.5 && son(i,5)< Distance_Wall +0.2
    %          %soundsc(signalopen,Fs);
    %          RIGHTDoorOpen=1
    %      end


 end
 pioneer_close(sp);
 
 %%
 