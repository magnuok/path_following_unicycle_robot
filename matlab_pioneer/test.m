% sp=serial_port_start
% pause(1);
% %% 
% pioneer_init(sp)
% pause(1); 
%%
pioneer_set_controls(sp, 200, 0);
t = zeros(500,1);
odemetry = zeros(500,3);
pause(1);
for i=1:500
    tic;
%     pause(0.1)
    odeme = pioneer_read_odometry;
    odemetry(i,1) = odeme(1);
    odemetry(i,2) = odeme(2);
    odemetry(i,3) = odeme(3);
    t(i) = toc;
    i
end

pioneer_set_controls(sp, 0, 0);