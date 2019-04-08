function pioneer_set_heading(sp,heading)

pioneer_sendmsg(sp, [12,splitint(heading)]);

end
