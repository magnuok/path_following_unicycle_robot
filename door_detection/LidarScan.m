% Get a LIDAR Scan
% Returns a range vector of 682 elements after a Lidar Scan from min step
% to max step.
% Range Values correspond from -120 to +120 degrees.
% Author- Shikhar Shrestha, IIT Bhubaneswar
%
% adapted to Octave- Joao Sequeira, 2019
% switch off the MATLAB variable if used in  Octave
% it assumes the instrument-control pkg was loaded before (most likely in the SeetupLidar function)
%
function [rangescan]=LidarScan(lidar)

    MATLAB = 1;

    proceed=0;
    while (proceed==0)
        if MATLAB  
            fprintf(lidar,'GD0044072500');
            pause(0.01);
            data=fscanf(lidar);
        else
            srl_write( lidar, "GD0044072500\r");
            pause(0.01);
            [data, nbytes] = srl_read( lidar, 10000);
        end
        if numel(data)==2134
            proceed=1;
        end
    end

    i=find(data==data(13));
    rangedata=data(i(3)+1:end-1);
    for j=0:31
        onlyrangedata((64*j)+1:(64*j)+64)=rangedata(1+(66*j):64+(66*j));
    end
    j=0;
    for i=1:floor(numel(onlyrangedata)/3)
        encodeddist(i,:)=[onlyrangedata((3*j)+1) onlyrangedata((3*j)+2) onlyrangedata((3*j)+3)];
        j=j+1;
    end
    for k=1:size(encodeddist,1)
        rangescan(k)=decodeSCIP(encodeddist(k,:));
    end

end