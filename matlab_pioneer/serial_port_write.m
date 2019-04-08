%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Escreve o conteudo do array msg na porta serie 
%
% Joao Sequeira, 2003
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function status = serial_port_write(sp, msg)

if strcmp(sp.TransferStatus, 'idle') | strcmp(sp.TransferStatus,'read')        
    fwrite(sp,msg,'uint8','async');
    status = 0;
else
    status = 1;
end
return