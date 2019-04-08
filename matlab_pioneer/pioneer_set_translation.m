function pioneer_set_translation(sp, distance)

pioneer_sendmsg(sp, [8,splitint(distance)]);

end
