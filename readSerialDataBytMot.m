function readSerialDataBytMot(src, ~)
  
    packetSize = 38; % Packet Size 

    while src.NumBytesAvailable >= packetSize  %packet size control
        rawData = read(src, packetSize, "uint8");

        % preamp - end check
        if rawData(1) == hex2dec('02') && rawData(end) == hex2dec('03')
            payload = rawData(2:end-1); % 36 byte payload
            %  F_msg 
            % 7 float + 2 int32 
            I1          = typecast(uint8(payload(1:4)),   'single'); % current on channel 1
            I2          = typecast(uint8(payload(5:8)),   'single'); % current on channel 2
            I3          = typecast(uint8(payload(9:12)),  'single'); % current on channel 3
            I4          = typecast(uint8(payload(13:16)), 'single'); % current on channel 4
            Vbus        = typecast(uint8(payload(17:20)), 'single'); % Bus voltage of DRV
            Vel_QENC    = typecast(uint8(payload(21:24)), 'single'); % optical Encoder data
            Pos_MagENC  = typecast(uint8(payload(25:28)), 'single'); % Magnetic Encoder data
            status    = typecast(uint8(payload(29:32)), 'int32');    % Fault control
            SaveStat    = typecast(uint8(payload(33:36)), 'int32');  % check for experiment finish

            % User Data init
            if isempty(fieldnames(src.UserData))
                src.UserData = struct( ...
                    'I1',[], 'I2',[], 'I3',[], 'I4',[], ...
                    'Vbus',[], 'Vel_QENC',[], 'Pos_MagENC',[], ...
                    'status',[], 'Count',0);
            end

            % Create structure for data
            src.UserData.I1(end+1) = I1;
            src.UserData.I2(end+1) = I2;
            src.UserData.I3(end+1) = I3;
            src.UserData.I4(end+1) = I4;
            src.UserData.Vbus(end+1) = Vbus;
            src.UserData.Vel_QENC(end+1) = Vel_QENC;
            src.UserData.Pos_MagENC(end+1) = Pos_MagENC;
            src.UserData.status(end+1) = status;
            src.UserData.Count = src.UserData.Count + 1;

            % for debug, the below line can be comment for fast comminication
            fprintf('Cnt:%04d | M:%d | I1=%.3f I2=%.3f I3=%.3f I4=%.3f | Vbus=%.2f | Vel=%.3f | Pos=%.3f | S=%d\n', ...
                src.UserData.Count, status, I1, I2, I3, I4, Vbus, Vel_QENC, Pos_MagENC, SaveStat);

            % save and show 
            if SaveStat == 999
                configureCallback(src, "off");

                Ts = 0.0025;
                t = (0:src.UserData.Count-1) * Ts;
                saveFolder = 'C:\Users\mcpne\Desktop\MulMot\Grafik'; % should be changed on different computer 
                if ~exist(saveFolder, 'dir'), mkdir(saveFolder); end
                matFile = fullfile(saveFolder, 'StepMotorData5.mat'); % save for post processing
                userData = src.UserData;
                userData.t = t;  % time vektor
                save(matFile, '-struct', 'userData');
               % fprintf('Data "%s" saved.\n', saveFolder);

                % figure
                figure(1); clf; hold on; 
                plot(t, src.UserData.I1, 'LineWidth', 1.2);
                plot(t, src.UserData.I2, 'LineWidth', 1.2);
                plot(t, src.UserData.I3, 'LineWidth', 1.2);
                plot(t, src.UserData.I4, 'LineWidth', 1.2);
                
                grid on;
                title('Phase Current '); xlabel('Time (s)'); ylabel('A');

                figure(2); clf; plot(t, src.UserData.Vbus, 'LineWidth', 1.2); grid on;
                title('Bus Voltage'); xlabel('Time (s)'); ylabel('V');

                figure(3); clf; plot(t, src.UserData.Vel_QENC, 'LineWidth', 1.2);
                hold on; plot(t, src.UserData.Pos_MagENC, 'LineWidth', 1.2);
                grid on;
                title('Encoders');
            end
        else
            flush(src);
            disp('Packet synchronization error.');
        end
    end
end
