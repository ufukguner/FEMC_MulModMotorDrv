%  MCCommand Send and Receive Data Script 
clc; clear;

% Serial Port Settings
PORT = 'COM22';  % change com port if different, check device manager
BAUD = 115200;   % it is not limit USB CDC , it is posible to reach 1 Khz refresh rate for F_msg

s = serialport(PORT, BAUD);
flush(s);
% Callback struct
s.UserData = struct('I1',[],'I2',[],'I3',[],'I4',[], ...
                    'Vbus',[],'Vel_QENC',[],'Pos_MagENC',[], ...
                     'status',[],'Count',0);


% Callback for 38 byte F_msg package
configureCallback(s, "byte", 38, @readSerialDataBytMot);

% MCCommand 
START_DELIM = uint8(30);  % preamble for protocol
PACKET_SIZE = 58;  % 1 start + 56 payload + 1 checksum

% MCCommand Alanları
Opcode   = uint8(1);     % Only one command, reserved for feature improvement
MotType  = uint8(2);     % 0: DC, 1: Step, 2: BLDC
H_Bridge = uint8(1);     % bridge configuration, not active, motor idendification used,reserved for feature improvement
Encoder  = uint8(0);     % optic encoder channel selecet 0:tim1 1:tim8 

Vel_Set  = single(15.0);     % velocity referance, not used, (radian/sec), reserved for feature improvement
Pos_Set  = single(2.0);      % position referance  (radian), active
Kp       = single(5);        % PID Kp parameteres, BLDC:5     Step 110
Ki       = single(0.55);     % PID Ki parameteres, BLDC:0.055 Step 70
Kd       = single(0.01);     % PID Kd parameteres, BLDC:0.01  Step 0
Ipk      = single(1);         % DRV current referenece, BLDC:1 Step:2
freq     = uint32(1000);      % Control loop refresh rate, micro second
S_Num    = uint32(2000);      % Experimenet sample number,  S_Num*2.5ms
MiroStep    = uint32(128);    % Step motor micro step size
Polepair = uint32(11);        % Pole pair, only BLDC/PMSM
Gain = single(0.50);          % Gain, only Step motor
StepSpeed =single(50);        % For open loop control of Stepmotor
GearRatio =single(1.0);       % motor reductor ratio, if no gear, must 1

% Create Payload  
payload = [ ...
    Opcode; ...
    MotType; ...
    H_Bridge; ...
    Encoder; ...
    typecast(Vel_Set,  'uint8')'; ...
    typecast(Pos_Set,  'uint8')'; ...
    typecast(Kp,       'uint8')'; ...
    typecast(Ki,       'uint8')'; ...
    typecast(Kd,       'uint8')'; ...
    typecast(Ipk,      'uint8')'; ...
    typecast(freq,     'uint8')'; ...
    typecast(S_Num,    'uint8')'; ...
    typecast(MiroStep,    'uint8')'; ...
    typecast(Polepair,    'uint8')'; ...
    typecast(Gain,    'uint8')'; ...
    typecast(StepSpeed,    'uint8')'; ...
    typecast(GearRatio,    'uint8')' ...
    ];

% start + Payload
frame_wo_checksum = [START_DELIM; payload];

% Calculate Checksum (sum of first 37 bytes), BCC for fast communication
Checksum = uint8(mod(sum(frame_wo_checksum), 256));
frame = [frame_wo_checksum; Checksum];

% Send command to system
write(s, frame, "uint8");
fprintf('\n Command sent (%d bytes)\n', numel(frame));
disp(dec2hex(frame)');


