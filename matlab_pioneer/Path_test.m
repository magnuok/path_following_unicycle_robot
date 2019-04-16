
% 
%sp=serial_port_start('COM5');
% pause(2);
%pioneer_init(sp);
t=200;
%figure(1);xlabel('x');ylabel('y');title('Odometry');
%figure(2);title('Sonar 1 and 7');
pioneer_set_controls(sp, 70, 0);

for i=1:t
    pause(1);
    odeme = pioneer_read_odometry;
    odemetry(i,1) = odeme(1)/1000;
    odemetry(i,2) = odeme(2)/1000;
    figure(1);plot(odemetry(i,1),odemetry(i,2),'x');xlabel('x');ylabel('y');title('Odometry');
    hold on;
    sonar=pioneer_read_sonars();
    son(i,1)=sonar(1);
    son(i,2)=sonar(7);
    
    figure(2);plot(i,son(i,1),'x');title('Sonar 1');
    hold on;
    figure(3);plot(i,son(i,2),'x');title('Sonar 7');
    hold on;
    if son(i,1)< 500
        pioneer_set_controls(sp, 0, -20);
        pause(0.2);
        pioneer_set_controls(sp, 70, 0);
    end
    if son(i,2)< 500
        pioneer_set_controls(sp, 0, +20);
        pause(0.2);
        pioneer_set_controls(sp, 70, 0);
    end
end


pioneer_close(sp);