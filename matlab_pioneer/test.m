% Serial port object = pioneer
% need to change a parameter inside here
sp = serial_port_start();
pioneer_init(sp);
pause(3);

% Move 4 meters. measure


pioneer_set_controls(sp, 200, 0);
pause(20);

pioneer_set_controls(sp, 0, 0);

pause(1);

pose = pioneer_read_odometry();

pioneer_close(sp)

% CHECK IF POSITIVE OR NEGATIV
theta_offsett = acos(pose(1,1)/4000)