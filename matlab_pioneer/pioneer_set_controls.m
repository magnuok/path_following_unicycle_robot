function pioneer_set_controls(sp, v, om)

pioneer_sendmsg(sp, [11, splitint(v)]);
pioneer_sendmsg(sp, [9,  splitint(om)]);

return
