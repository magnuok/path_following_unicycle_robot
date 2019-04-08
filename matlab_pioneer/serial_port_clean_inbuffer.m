%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Limpa o buffer de entrada da porta serie e
% o read_buffer local
%
% Joao Sequeiram 2003
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function serial_port_clean_inbuffer(obj)

global read_buffer;

%disp(['Cleaning ', num2str(obj.BytesAvailable), ' bytes']);

read_buffer = [];
if obj.BytesAvailable>0
    fread(obj,obj.BytesAvailable,'uint8');
end
return
