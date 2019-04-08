%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Inicializa o Pioneer
%
% Joao Sequeira, 2003
% Rodrigo Ventura, 2007
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function pioneer_init(sp)

% CONFIG %
timer_period = 0.10; %secs
%

global pioneer_timer;
global pioneer_lock;

pioneer_lock=0;

serial_port_clean_inbuffer(sp);

pioneer_sendmsg(sp,[0]);
pioneer_getmsg(sp);
disp('SYNC0');
pioneer_sendmsg(sp,[1]);
pioneer_getmsg(sp);
disp('SYNC1');
pioneer_sendmsg(sp,[2]);
info=pioneer_getmsg(sp);
disp('SYNC2, robot info:');

[m,n]=size(info);
s='';
for i=2:n
    if info(i)==0
        disp(s);
        s='';
    else
        s=[s, char(info(i))];
    end
end

% OPEN
pioneer_sendmsg(sp,[1]);

% MOTORS ON
pause(0.5);
pioneer_sendmsg(sp,[4,splitint(1)]);

pioneer_timer = timer('ExecutionMode', 'fixedRate', 'Period', timer_period, ...
                        'TimerFcn', {@pioneer_handler,sp}, 'Name', 'Pioneer_timer');
start(pioneer_timer);

return
