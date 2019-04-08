function n = parseint(data)
    n = data(1) + 2^8*data(2);
    if data(2)>127
        n = n-2^16;
    end
return
    