clear
close all;
sp=serial_port_start('COM5')
pause(2);
pioneer_init(sp)
pause(2);
t=200;


%[signalclose, Fs]=audioread('fugee.wav');
%[signalopen, Fs]=audioread('fugee.wav');


%%
% Use the laser:
% Door will correspond to a jump in the readings
% First derivative go crazy
% Use sin/cos/tg((distance to crazy first derivative)/distance to wall) 
% Correct the odometry this way

%%






%pioneer_set_controls(sp, 220, 0);
 status =2; actual=1;
 pause(0.2);
 
 %Status = 0  Leaving the room;
 %Status=1 First corridor;
 i=0;
 son=zeros(20000,8);
 odemetry=zeros(20000,3);
 while status < 4
    i=i+1;
    sonar=pioneer_read_sonars();
    son(i,1)=sonar(1)/1000;son(i,2)=sonar(2)/1000;
    son(i,3)=sonar(3)/1000;son(i,4)=sonar(4)/1000;
    son(i,5)=sonar(5)/1000;son(i,6)=sonar(6)/1000;
    son(i,7)=sonar(7)/1000;son(i,8)=sonar(8)/1000;
%     %This figure shows the last 3 Sonar scans
%     figure(2);title('Sonar 1');
%     xlabel('Sonar');ylabel('Distance(m)');
%     aux=i;
%     plot(1,son(aux,1),'x');
%     hold on;
%     plot(2,son(aux,2),'x');plot(3,son(aux,3),'x');plot(4,son(aux,4),'x');
%     plot(5,son(aux,5),'x');plot(6,son(aux,6),'x');plot(7,son(aux,7),'x');plot(8,son(aux,8),'x');
%     legend('Sonar 1','Sonar 2','Sonar 3','Sonar 4','Sonar 5','Sonar 6','Sonar 7','Sonar 8');
%     hold off;

% CLOSED DOOR
    if son(i,4)< 0.5 && son(i,5)< 0.5
        %soundsc(signalclose,Fs);
        DoorClosed=1
        pause(30);
    end
% OPEN DOOR
    if son(i,4)> 0.7 && son(i,5)> 0.7
        %soundsc(signalopen,Fs);
        DoorOpen=1
    end
    if son(i,4)< 0.7 && son(i,5)> 1
        %soundsc(signalopen,Fs);
        LEFTDoorOpen=1
    end
    
 
% RIGHT OPENING DOOR
   % if son(i,3)> 0.7 || son(i,6)> 0.7
        %soundsc(signalopen,Fs);
    %end
% LEFT OPENING DOOR

 end
 pioneer_close(sp);
 
 %%
 