%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Receives a packet from the pioneer, via serial port
%
% Rodrigo Ventura, 2007
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function packet = pioneer_recvmsg(sp)

global read_buffer;

serial_port_read_bulk(sp);

packet=[];

[m,n]=size(read_buffer);

state=0;

for i=1:n
    switch state
        case 0
            % first framing byte
            if read_buffer(i)==250
                state=1;
            end
        case 1
            % second framing byte
            if read_buffer(i)==251
                state=2;
            else
                state=0;
            end
        case 2
            % byte count
            count=read_buffer(i);
            if count < 2
                % bad byte count, ignore packet
                state=0;
                disp('[PACKET ERROR: count<2, discarding]');
            elseif n >= i+count
                % got whole packet
                packet = read_buffer(i+1:i+count);
                read_buffer = read_buffer(i+count+1:n);
                csum = pioneer_chksum(packet(1:count-2));
                if csum == packet(count-2+1:count)
                    % packet is ok, return then
                    packet = packet(1:count-2);
                    %disp(sprintf('[GOOD PACKET: payload %d bytes]', count)); 
                    return
                else
                    % bad checksum!
                    state=0;
                    disp('[PACKET ERROR: bad checksum, discarding]');
                end
                return
            else
                % incomplete packet
                return
            end
    end
end
return