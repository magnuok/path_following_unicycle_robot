%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Waits for, and receives a packet from the pioneer, via serial port
%
% Rodrigo Ventura, 2007
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function packet = pioneer_getmsg(obj)

while 1
    packet = pioneer_recvmsg(obj);
    if ~isempty(packet)
        return
    else
        pause(0.1);
    end
end

return
