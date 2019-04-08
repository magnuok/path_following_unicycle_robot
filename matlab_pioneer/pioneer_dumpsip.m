
function pioneer_dumpsip(data)

typ=data(1);
x=parseint(data(2:3));
y=parseint(data(4:5));
th=parseint(data(6:7));

battery=data(12);
nsonars=data(20);

disp(sprintf('SIP:\n  type=%d\n  xpos=%d\n  ypos=%d\n  theta=%d\n', typ, x, y, th))
disp(sprintf('  battery=%f\n  #sonars=%d\n', battery/10, nsonars))

for i=1:nsonars
    ptr = 21+(i-1)*3;
    sid = data(ptr);
    rng = parseuint(data(ptr+1:ptr+2));
    disp(sprintf('    [%d] --> %d\n', sid, rng));
end

return
