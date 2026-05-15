clear;
clc;
close all;

filterCase = 1; % 1: LSM6DS3 IMU (arduino_madgwick_imu.ino) 2: Xsens IMU (mti3_madgwick_imu.ino)
port       = "COM3";
baud       = 115200;
duration   = 4 * 60;     % 4 minutes

names    = {'LSM_IMU', 'XSENS_IMU'};
caseName = names{filterCase};
outFile  = sprintf('rpy_%s.mat', caseName);

delete(serialportfind);
s = serialport(port, baud);
configureTerminator(s, "LF");
pause(3);
flush(s);

fprintf('--- Logging %s for %d seconds ---\n', caseName, duration);
fprintf('Keep sensor STATIONARY.\n\n');

tLog = []; rollLog = []; pitchLog = []; yawLog = [];
k = 0;
t0 = tic;

while toc(t0) < duration
    try
        line = readline(s);
        vals = str2double(strsplit(strtrim(line), ','));
        if length(vals) ~= 4 || any(isnan(vals)), continue; end

        q = vals / norm(vals);

        roll  = atan2(2*(q(1)*q(2) + q(3)*q(4)), 1 - 2*(q(2)^2 + q(3)^2));
        pitch = asin(max(-1, min(1, 2*(q(1)*q(3) - q(4)*q(2)))));
        yaw   = atan2(2*(q(1)*q(4) + q(2)*q(3)), 1 - 2*(q(3)^2 + q(4)^2));

        k = k + 1;
        tLog(k,1)     = toc(t0);
        rollLog(k,1)  = rad2deg(roll);
        pitchLog(k,1) = rad2deg(pitch);
        yawLog(k,1)   = rad2deg(yaw);
    catch
        continue;
    end
end

clear s;

% yaw drift (linear fit after 10s)
idx  = tLog > 10;
pFit = polyfit(tLog(idx), yawLog(idx), 1);
driftRate = pFit(1);    % deg/s

fprintf('\n===== %s =====\n', caseName);
fprintf('  Yaw drift rate: %.4f deg/s\n', driftRate);
fprintf('  Total yaw change (end - start): %.2f deg\n', yawLog(end) - yawLog(1));

% rpy stats (after 10s)
fprintf('\n  Roll  : mean = %.4f deg, std = %.4f deg\n', ...
    mean(rollLog(idx)),  std(rollLog(idx)));
fprintf('  Pitch : mean = %.4f deg, std = %.4f deg\n', ...
    mean(pitchLog(idx)), std(pitchLog(idx)));

figure('Color','w','Position',[100 100 1000 700]);

subplot(3,1,1);
plot(tLog, rollLog); grid on;
title('Roll');
ylabel('deg'); ylim([-180 180]);

subplot(3,1,2);
plot(tLog, pitchLog); grid on;
title('Pitch');
ylabel('deg'); ylim([-90 90]);

subplot(3,1,3);
plot(tLog, yawLog); grid on;
title('Yaw');
ylabel('deg'); xlabel('time (s)'); ylim([-180 180]);

save(outFile, 'tLog', 'rollLog', 'pitchLog', 'yawLog', ...
              'caseName', 'duration', 'driftRate');
fprintf('\nSaved: %s\n', outFile);