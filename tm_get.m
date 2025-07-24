function [speed, incline] = tm_get(obj)

% Read 32 byte data from tcpip object
packet	= fread(obj, 32, 'uint8');

% Parse data
packet  = uint16(packet); % convert to 16 bit
speed	= bitshift(packet(2+[0:3]*2), 8); % high 8 bits
speed   = bitor(speed, packet(3+[0:3]*2)); % low 8 bits
speed   = typecast(speed, 'int16'); % sign
speed   = double(speed) / 1000.0; % decimals, [m/s], vector

incline = bitshift(packet(10), 8);
incline = bitor(incline, packet(11));
incline	= typecast(incline, 'int16');
incline	= double(incline) / 100.0; % [deg], scalar

end

