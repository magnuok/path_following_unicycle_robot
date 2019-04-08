%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Sends a packet to the pioneer, via serial port
%
% Rodrigo Ventura, 2007
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function status = pioneer_sendmsg(sp, msg)

global pioneer_lock;

[m,n]=size(msg);
packet = [250, 251, n+2, msg, pioneer_chksum(msg)];

pioneer_lock=1;
while strcmp(sp.TransferStatus,'idle')==0
    % spinlock ;)
end
status = serial_port_write(sp, packet);
pioneer_lock=0;

return