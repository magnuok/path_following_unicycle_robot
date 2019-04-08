function data = splitint(x)

if x<0
    x = uint16(x + 2^16);
end
lb=bitand(x, 2^8-1);
hb=bitshift(x, -8);

data=double([59, lb, hb]);

return
