function serial_port_stop(sp)

stopasync(sp);  %%% Stop all async communication
fclose(sp);     %%% Disconnect from port
delete(sp);
clear sp;      %%% Remove port from workspace
return