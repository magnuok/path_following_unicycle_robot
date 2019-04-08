%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Handler for event BytesAvailable
% To be used with the read_message function
% 
% The disp call are just for debug purposes
%
% Joao Sequeira, 2003
% Rdrigo Ventura, 2007
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function serial_port_read_bulk(obj, event)

global read_buffer;

if obj.BytesAvailable>0
    aux = fread(obj,obj.BytesAvailable,'uint8');
    read_buffer = [read_buffer, aux' ];
end

return