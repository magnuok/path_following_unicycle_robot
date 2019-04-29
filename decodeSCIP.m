% Function to decode range information transmitted using SCIP2.0 protocol.
% Works for only two and three bit encoding.
% Author- Shikhar Shrestha, IIT Bhubaneswar
function rangeval=decodeSCIP(rangeenc)
% Check for 2 or 3 Character Encoding
if rangeenc(1)=='0' && rangeenc(2)=='0' && rangeenc(3)=='0'
    rangeval=0;
    return;
end
if rangeenc(1)=='0'
    dig1=((rangeenc(2)-'!')+33);
    dig2=((rangeenc(3)-'!')+33);
    dig1sub=dig1-48;
    dig2sub=dig2-48;
    dig1bin=dec2bin(dig1sub,6);
    dig2bin=dec2bin(dig2sub,6);
    rangeval=bin2dec([dig1bin dig2bin]);
    return;
else
    dig1=((rangeenc(1)-'!')+33);
    dig2=((rangeenc(2)-'!')+33);
    dig3=((rangeenc(3)-'!')+33);
    dig1sub=dig1-48;
    dig2sub=dig2-48;
    dig3sub=dig3-48;
    dig1bin=dec2bin(dig1sub,6);
    dig2bin=dec2bin(dig2sub,6);
    dig3bin=dec2bin(dig3sub,6);
    rangeval=bin2dec([dig1bin dig2bin dig3bin]);
    return;
    
end
    
end