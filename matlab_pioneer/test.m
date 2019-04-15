% % sp=serial_port_start
% % pause(3);
% % % 
% % pioneer_init(sp)
% % pause(3); 
% % 
pioneer_set_controls(sp, 200, 0);
t = zeros(1000,1);
odemetry = zeros(1000,3);
pause(1);
for i=1:1000
    tic;
    pause(0.0064)
    odeme = pioneer_read_odometry;
    odemetry(i,1) = odeme(1);
    odemetry(i,2) = odeme(2);
    odemetry(i,3) = odeme(3);
    t(i) = toc;
    i
end
tsum(t, odemetry)
pioneer_set_controls(sp, 0, 0);