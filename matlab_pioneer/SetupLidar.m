% Setup Lidar 
% % Configures Serial Communication and Updates Sensor Communication to
% SCIP2.0 Protocol.
% % Checks Version Information and switches on Laser.
% Author- Shikhar Shrestha, IIT Bhubaneswar
%
% adapted to Octave- Joao Sequeira, 2019
% switch off the MATLAB variable if used in  Octave
%

function lidar = SetupLidar()
MATLAB = 1;


if MATLAB
    lidar = serial('COM10','baudrate',115200);
    set(lidar,'Timeout',0.1);   
    set(lidar,'InputBufferSize',40000);
    set(lidar,'Terminator','CR');
    pause(0.1);
    fopen(lidar);
    pause(0.1);
    fprintf(lidar,'SCIP2.0');
    pause(0.1);
    fscanf(lidar);
    fprintf(lidar,'VV');
    pause(0.1);
    fscanf(lidar)
    fprintf(lidar,'BM');
    pause(0.1);
    fscanf(lidar)
    
else
    pkg load instrument-control
    
    % keep this weird name structure for the COM port
    %
    lidar = serial("\\\\.\\COM10", 115200);
    set( lidar, 'bytesize', 8);
    set( lidar, 'parity', 'n');
    set( lidar, 'stopbits', 1);
    set( lidar, 'requesttosend', 'off');
    set( lidar, 'dataterminalready', 'off');
    set( lidar, 'timeout', 1);    % in 1/10 of second
    srl_flush(lidar);
 
    % the strings to be written can't be enclosed in ''
    % they must be enclosed in ""
    
    srl_write(lidar, "SCIP2.0\r");
    pause(0.1);
    [data, nread] = srl_read(lidar, 1000);
    char(data)
    
    srl_write(lidar, "VV\r");    
    pause(0.1);
    [data, nread] = srl_read(lidar, 1000);
    char(data)
    
    srl_write(lidar, "BM\r");
    pause(0.1);
    [data, nread] = srl_read(lidar, 1000);
    char(data)
    
    printf('lidar set\n')
    fflush(stdout)
    
    % in case of testing this function don't forget to srl_close after running this
    % otherwise the device is busy
%    srl_close(lidar)
end

end
