% Purpose:
% This program synchronously starts the Vicon/Nexus, Bertec/Treadmill and
% Exoskeleton, and controls the treadmill to run at specific incline and
% speed for level walking, level running, ramp ascent and ramp descent
% activities. Four speed perturbations will be imposed in each locomotion 
% bout, with each two for speed forward (increase) and speed backward
% (decrease). 
%
%
% Procedures:
% (1) Configure and enable the Nexus UDP and the Bertec TCP function;
% (2) Run this program and wait for a start signal from robot via a TCP
% connection to trigger a synchronized start of all devices.
% (3) The program will terminate automatically when a termination condition
% is satisfied or once receiving a STOP signal from Robot.
%
%
% Check Items:
% (1) Check port number of TCP and UDP connections;
% (2) Check the content of Nexus 'Trail Description';
% (3) Check the variable values of 'PSpeed' and 'PSlope';
%
%
% The program has been tested on Matlab R2022b, Vicon Nexus 2.10.3, and
% Bertec Treadmill Control Panel 1.8.8.1.
%

clear
clc


%% -----------------------------Pre-define-----------------------------
actType = 'LW'; % LW--level walking, RN--running, RA--ramp ascent, RD--ramp descent
exoCond = 'ACT'; % ACT--active, PAS--passive, NOE--nonExoskeleton (blind to the subjects)
sbjMass = 70; % [kg]
patient = 'tanxiaowei';



%% -----------------------------TCP setting (Robot-->Matlab)-----------------------------
while true
    RobotTCP = instrfindall('Type','tcpip'); % instrument find all
    if isempty(RobotTCP)
        IP   = '192.168.0.11';  % LocalIP
        Port = 8080;            % LocalPort
        RobotTCP  = tcpserver(IP, Port);
        set(RobotTCP, 'ByteOrder', 'little-endian'); % difference between C++ and Matlab
        break;
    else
        disp("[MSG]: RobotTCP was existed and is going to be closed");
        fclose(RobotTCP);
        delete(RobotTCP);
    end
    pause(0.01/1000); % 0.01ms
end



%% -----------------------------UDP setting (Matlab-->Nexus)-----------------------------
% define the path to store the trial data [NOT APPLICABLE! The data are always stored in the Nexus's current path]
classif = 'shankAngleControl'; % third-level directory
% patient = 'NA'; % fourth-level directory
session = datestr(now, 'yyyy-mm-dd'); % fifth-level directory
trlAdre = ['"E:\Vicon\' classif '\' patient '\' session '\"']; % absolute path

% define the trial name
trlName = datestr(now, 'hh-MM-SS'); % be unique otherwise the previous one will be overlapped

% add a description for the trial, for example, [Name+Gender+Birth Date+Height+Mass]
% e.g., Tanxiaowei+Male+19910326+1.80m+70kg
% trlDesc = '"TANxiaowei+Male+19910326+1.80m+70kg"';
trlDesc = ['"' patient '+' actType '+' exoCond '+' num2str(sbjMass) 'kg"'];

% define start and stop trigger messages
MSGSTART = ['<?xml version="1.0" encoding="UTF-8" standalone="no" ?>'...
    '<CaptureStart>'...
    '<Name VALUE="' trlName '"/>'...
    '<Notes VALUE=""/>'...
    '<Description VALUE=' trlDesc '/>'...
    '<DatabasePath VALUE=' trlAdre '/>'...
    '<Delay VALUE="0"/>'...
    '<PacketID VALUE="' num2str(randi(1e5)) '"/>'... % packets with duplicate ID's are ignored by Nexus
    '</CaptureStart>'];

MSGSTOP  = ['<?xml version="1.0" encoding="UTF-8" standalone="no" ?>'...
    '<CaptureStop RESULT="SUCCESS">'...
    '<Name VALUE="' trlName '"/>'... % the same as in the STARTMSG
    '<DatabasePath VALUE=' trlAdre '/>'... % the same as in the STARTMSG
    '<Delay VALUE="0"/>'...
    '<PacketID VALUE="' num2str(randi(1e5)) '"/>'... % packets with duplicate ID's are ignored by Nexus
    '</CaptureStop>'];

% connect to the Nexus via udp
while true
    NexusUDP = instrfindall('Type','udp'); % instrument find all
    if isempty(NexusUDP)
        IP   = '192.168.140.1';  % RemoteIP
        Port = 4040;            % RemotePort; 4040 should be pre-set in Nexus
        NexusUDP  = udp(IP, Port);
        break;
    else
        disp("[MSG]: NexusUDP was existed and is going to be closed");
        fclose(NexusUDP);
        delete(NexusUDP);
    end
    pause(0.01/1000); % 0.01ms
end



%% -----------------------------Open TCP and Wait Robot Signal--------------------------------
if (strcmp(exoCond, 'ACT')) % connect to robot only in the ACTIVE condition
    disp('[MSG]: Waiting for a client tcp/ip connection...');
    try
        % keep blocking until a client is connected (unrelated to 'timeout' variable)
        fopen(RobotTCP);
        disp('[MSG]: A client has connected');
    catch
        disp('[ERROR]: Open tcp/ip port failed');
        return;
    end
    
    disp('[MSG]: Waiting for datastream from the client');
    while true
        while ~RobotTCP.BytesAvailable
            pause(0.01/1000); % 0.01ms
        end
        
        dataStreamFromRobotTCP = fread(RobotTCP, 1, 'char');
        % 0--stop, 1--start, 2--stance£¬3--swing£¬4~255--reserved
        if char(dataStreamFromRobotTCP) == '1'
            tic; % set Matlab timestamp to synchronize with robot
            disp('[MSG]: Received: START');
            
            % tell robot the current subject's mass [kg] to calculate the
            % desired assistive force magnitude
            fwrite(RobotTCP, num2str(sbjMass), 'char');
            break;
        else
            disp(['[MSG]: Received data are ', num2str(dataStreamFromRobotTCP), '(Ascii), wait to receive a new signal']);
        end
    end
end



%% -----------------------------Start Nexus & Connected Devices--------------------------------
fopen(NexusUDP);
% send Nexus-start trigger message
fwrite(NexusUDP, MSGSTART);
TicNexus = tic;
disp("[MSG]: Nexus-start trigger message sending finished");



%% -----------------------------Main Function--------------------------------
% don't forget the 0.25s delay in startup of Nexus (which can be found by comparing the total time)
DelayinNexus = toc - toc(TicNexus);
disp(['[MSG]: Nexus timestamp is later ', num2str(DelayinNexus), 's (+0.25s) than the Robot timestamp']);
mainFnc(RobotTCP, NexusUDP, MSGSTOP, actType, exoCond);



%% -----------------------------Main Function--------------------------------
function mainFnc(RobotTCP, NexusUDP, MSGSTOP, actType, exoCond)
% -------------------Connect to Bertec Treadmill-------------------
try
    Bertec = tm_connect();
    disp('[MSG]: Bertec treadmill is connected');
catch
    cleanUp(RobotTCP, NexusUDP, MSGSTOP, Bertec, exoCond);
end

% -------------------Set CleanUp Function-------------------
cleanupObj = onCleanup(@()cleanUp(RobotTCP, NexusUDP, MSGSTOP, Bertec, exoCond));

% -------------------Individual Belt Speed Paras-------------------
[~, BSlope] = tm_get(Bertec); % [deg] get Bertec slope angle. The belt
% slope should be pre-set as the designated slope angle.

if BSlope < 0
    disp('[ERROR]: Current incline value is less than zero');
    return;
end

if strcmp(actType, 'LW') || strcmp(actType, 'RN')
    if abs( BSlope - 0 ) > 0.2 % [deg]
        disp('[ERROR]: Mismatch between current and designated incline value');
        return;
    end
    
    if strcmp(actType, 'LW')
        BSpeed = 1.2; % WALK: 1.2m/s
    else
        BSpeed = 2.2; % RUN: 2.2m/s
    end
    
elseif strcmp(actType, 'RA') || strcmp(actType, 'RD')
    if abs( BSlope - 10 ) > 0.2 % [deg]
        disp('[ERROR]: Mismatch between current and designated incline value');
        return;
    end
    
    if strcmp(actType, 'RA')
        BSpeed = 1.2;  % RAMP ASCENT: 1.2m/s
    else
        BSpeed = -1.2; % RAMP DESCENT: -1.2m/s
    end
    
else
    disp('[ERROR]: Error input of "actType" ');
    return;
end

% -------------------If there is already a STOP signal sent from Robot-------------------
if (strcmp(exoCond, 'ACT'))
    if RobotTCP.BytesAvailable
        dataFromRobotTCP = fread(RobotTCP, 1, 'char');
        
        if dataFromRobotTCP == '0' % STOP
            disp("[MSG]: Received STOP");
            return
        end
    end
end

% -------------------Wait for Subject Preparation-------------------
for i = 1:5
    
    % if there is already a STOP signal sent from Robot
    if (strcmp(exoCond, 'ACT'))
        if RobotTCP.BytesAvailable
            dataFromRobotTCP = fread(RobotTCP, 1, 'char');
            
            if dataFromRobotTCP == '0' % STOP
                disp("[MSG]: Received STOP in preparation period");
                return
            end
        end
    end
    
    sound( sin(2*pi*25*(1:200)/100) ); % short beep sound
    pause(1); % [s] for preparation
end
sound( sin(2*pi*25*(1:5000)/100) ); % long beep sound

% -------------------Set Initial Belt Speed-------------------
tm_set(Bertec, BSpeed, 0.5, BSlope);

% -------------------Randomize the cycles during which perturbations will be imposed-------------------
rng("shuffle");
PBCycles = randi([10, 60], 1, 4); % four random cycles that will be imposed with speed perturbations
% check if there are adjacent perturbed cycles. Perturbations are not
% allowed to occur in consecutive gait cycles.
while ismember( 1, diff( sort(PBCycles) ))
    PBCycles = randi([10, 60], 1, 4); % regenerate the number index of perturbed cycles
end

PBCyclesUP = sort( PBCycles(1:2) ); % two for upward-speed perturbation
% PBCyclesDN = sort( PBCycles(3:4)) ; % two for downward-speed perturbation [Variable Not Used]

% -------------------End Once after the End-Cycle or Received a STOP Signal from Robot-------------------
cycle = 1;
period_tic = tic;
End_cycle  = 60;
speedTrackingDelay = 0.12; % [s] Bertec belt speed tracking delay

while cycle < End_cycle % only the left-side
    % wait data from Robot
    while ~RobotTCP.BytesAvailable
        pause(0.1/1000); % 0.1ms
    end
    
    % read data
    dataFromRobotTCP = fread(RobotTCP, 1, 'char');
    % 0--stop, 1--start, 2--stance, 3--swing, 4~255--reserved
    switch dataFromRobotTCP
        case '0'
            disp("[MSG]: Received: STOP");
            break; % jump out the 'while' loop
        case '1'
            % disp("[MSG]: Received: START");
        case '2'
            % disp("[MSG]: Received: STANCE");
            
            cycle  = cycle + 1; % left-side gait cycle number
            period = toc(period_tic);  % gait period
            period_tic = tic;
            
            %----------for display----------
            des = min( PBCycles(PBCycles >= cycle) );
            if isempty(des)
                PBstr = 'NULL';
            elseif ismember(des, PBCyclesUP)
                PBstr = 'Up';
            else
                PBstr = 'Down';
            end
            disp(['[MSG]: Current Cycle: No. ',     num2str(cycle), ...
                9, 'Last Cycle Period: ',           num2str(period), 's', ...
                9, 'Next Perturbed Cycle£ºNo. ',    num2str(des), '(', PBstr ')']);
            
            %----------should a perturbation be imposed in the current gait cycle----------
            if ismember(cycle, PBCycles)
                %----------design belt speed profile----------
                P01 = 0.15 * period; % stage #1: started at 15% gait period
                P02 = 0.2; % [s] stage #2: time duration for the speed perturbation
                P03 = 0.2; % [s] stage #3: this value is unimportant
                
                % upward or downward perturbation
                if ismember(cycle, PBCyclesUP)
                    PerturbationSpeed = BSpeed * 1.8; % add 80%
                else
                    PerturbationSpeed = BSpeed * 0.2; % minus 80%
                end
                
                % design speed profile for the current gait cycle
                v = [BSpeed, PerturbationSpeed, BSpeed]; % [m/s]
                t = [P01, P02, P03]; % [s]
                [p_t, p_v] = tm_speedProfileDesign(v, t, 200); % 200Hz sampling rate
                
                %----------calculate and send belt speed command----------
                ind = 1;
                SendTime = toc(period_tic) + speedTrackingDelay;
                while SendTime < max(p_t)
                    % find the index in the speed profile w.r.t. the
                    % current time index
                    SendTime = toc(period_tic) + speedTrackingDelay;
                    [~, indc] = min(abs(p_t - SendTime));
                    if ind ~= indc
                        ind = indc;
                    else
                        pause(0.01/1000);  % 0.01ms
                        continue;
                    end
                    
                    % send belt speed command
                    tm_set(Bertec, p_v(ind), 25, BSlope); %  acc = 25m/s2
                end
            end
            
        case '3'
            % disp("[MSG]: Received: SWING");
        otherwise
            disp("[MSG]: Received: WRONG SIGNAL");
    end
end

end



%% -----------------------------Cleanup Function--------------------------------
function cleanUp(RobotTCP, NexusUDP, MSGSTOP, Bertec, exoCond)

%-------------------zero and close Bertec-------------------
try
    % zero speed, keep the slope angle, and disconnect
    [~, BSlope] = tm_get(Bertec); % [deg] get the Bertec slope angle
    BSlope=abs(BSlope);
    tm_set(Bertec, 0, 0.25, BSlope);
    
    pause(100/1000); % 100 ms, otherwise the above command may be sent unsuccesfully.
    tm_disconnect(Bertec);
    disp("[MSG]: Bertec Treadmill Disonnected");
catch
end

%-------------------close robot program-------------------
if (strcmp(exoCond, 'ACT')) % connect to robot only in the ACTIVE condition
    try
        fwrite(RobotTCP, '0', 'char') % send STOP signal to Robot
        disp("[MSG]: Robot-stop signal sending finished");
        
        pause(10/1000); % 10ms
        delete(RobotTCP);
    catch
    end
end

%-------------------close Nexus-------------------
try
    fwrite(NexusUDP, MSGSTOP); % send STOP signal to Nexus
    disp("[MSG]: Nexus-stop signal sending finished");
    
    pause(10/1000); % 10ms
    delete(NexusUDP);
catch
end

disp("[MSG]: cleanUp function completed");
clear RobotTCP NexusUDP;
end