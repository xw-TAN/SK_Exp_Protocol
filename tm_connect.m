function [object] = tm_connect()
% Open the connection (do it only once at start of the control session)
object = tcpip('localhost', 4000,...
    'NetworkRole', 'client',...
    'Name', 'BertecTreadmill',...
    'Timeout', 5);

% Specify the input and output buffer size
set(object, 'InputBufferSize', 32, 'OutputBufferSize', 64);

% Specify the readCallBack function
% set(object, ...
%     'BytesAvailableFcnCount', 32,...
%     'BytesAvailableFcnMode', "byte",...
%     'BytesAvailableFcn',@tm_getCallback);

% Open the tcpip object
fopen(object);
% pause(10/1000); % pause 10ms

end

