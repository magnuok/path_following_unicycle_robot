%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Fecha comms com o Pioneer
%
% Rodrigo Ventura, 2007
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function pioneer_close(sp)

global pioneer_timer;

stop(pioneer_timer);
pioneer_sendmsg(sp,[2]);
pioneer_digest(sp);

return
