function time = tsum(t, odemetry)
    time = zeros(length(t),1);
    a=1;
    for i = 1 :length(t)
       if odemetry(i)==odemetry(i+1)
        time(a) = time(a) + t(i);
       end
       if odemetry(i)~=odemetry(i+1)
         a=a+1;
       end
    end
end