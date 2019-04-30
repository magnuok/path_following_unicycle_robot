% Serial port object = pioneer
% need to change a parameter inside here
sp = serial_port_start();
pioneer_init(sp);
pause(2);

pause(0.1);

pioneer_set_controls(sp, 0, 0);
pause(0.3);
%forward
pioneer_set_controls(sp, 300, 0);
pause(1.433333);
pioneer_set_controls(sp, 0, 0);
pause(0.1);
%turn
pioneer_set_controls(sp, 0, -85);
pause(1);
pioneer_set_controls(sp, 0, 0);

% Check if door is open here
pause(3);

% turn back
pioneer_set_controls(sp, 0, 85);
pause(1);
pioneer_set_controls(sp, 0, 0);
pause(0.1);
%backward
pioneer_set_controls(sp, -300, 0);
pause(1.433333);
pioneer_set_controls(sp, 0, 0);

pioneer_close(sp);