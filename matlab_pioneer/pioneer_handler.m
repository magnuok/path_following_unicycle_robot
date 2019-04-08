function pioneer_handler(obj, event, sp)

global pioneer_lock;

%disp('handler called');

pioneer_digest(sp);

if pioneer_lock==0
    pioneer_sendmsg(sp, [0]);
    %disp('[PULSE]');
end

end
