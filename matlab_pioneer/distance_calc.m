function [distance_to_wall]=distance_calc(scan,side)

    % Independent for each side
    if side == -1 %Right side
        scan_aux=scan(40:125);
    elseif side== 1
        scan_aux=scan(547:627);
    end
    
    %Reading from the scan and finding the distance to wall
    for l=1:1:length(scan_aux)
        if scan_aux(l) < 10
            scan_aux(l)=5000;
        end
    end
    distance_to_wall = min(scan_aux)/1000;

end