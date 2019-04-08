%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Computes packet checksum, according to ARIA rules
%
% Rodrigo Ventura, 2007
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function result = pioneer_chksum(msg)

[m,n]=size(msg);
b=0;
s=uint32(0);
if mod(n,2)==1
    nn=n-1;
else
    nn=n;
end
for i=1:nn
    c=uint32(msg(i));
    if b==0
        s = s + bitshift(c,8);
        b = 1;
    else
        s = s + c;
        b = 0;
    end
    s = bitand(s,2^16-1);
end
if mod(n,2)==1
    s = bitxor(s,msg(n));
end

result = [bitshift(s,-8), bitand(s,2^8-1)];
return
