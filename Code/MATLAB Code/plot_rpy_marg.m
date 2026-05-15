clear;
clc;
close all;

%  For Xsens MARG filter (run mti3_madgwick_marg.ino and make sure w_bx,w_by,w_bz also get printed)

port     = "COM3";
baud     = 115200;
duration = 60;
outFile  = 'beta-0.1_zeta-def.mat';

delete(serialportfind);
s = serialport(port, baud);
configureTerminator(s, "LF");
s.Timeout = 4;
pause(3);
flush(s);

fprintf('--- Logging XSENS_MARG for %d seconds ---\n', duration);
fprintf('Keep sensor STATIONARY.\n\n');

tLog = []; rollLog = []; pitchLog = []; yawLog = [];
wbxLog = []; wbyLog = []; wbzLog = [];
k = 0;
t0 = tic;

while toc(t0) < duration
    try
        line = readline(s);
        vals = str2double(strsplit(strtrim(line), ','));
        if length(vals) ~= 7 || any(isnan(vals)), continue; end

        q   = vals(1:4) / norm(vals(1:4));
        w_b = vals(5:7) * 180/pi;   % rad/s -> deg/s

        roll  = atan2(2*(q(1)*q(2) + q(3)*q(4)), 1 - 2*(q(2)^2 + q(3)^2));
        pitch = asin(max(-1, min(1, 2*(q(1)*q(3) - q(4)*q(2)))));
        yaw   = atan2(2*(q(1)*q(4) + q(2)*q(3)), 1 - 2*(q(3)^2 + q(4)^2));

        k = k + 1;
        tLog(k,1)     = toc(t0);
        rollLog(k,1)  = rad2deg(roll);
        pitchLog(k,1) = rad2deg(pitch);
        yawLog(k,1)   = rad2deg(yaw);
        wbxLog(k,1)   = w_b(1);
        wbyLog(k,1)   = w_b(2);
        wbzLog(k,1)   = w_b(3);
    catch
        continue;
    end
end

clear s;

% stats after 10 seconds
idx = tLog > 10;

fprintf('\n===== XSENS_MARG =====\n');
fprintf('  Roll  : mean = %.4f deg, std = %.4f deg\n', ...
    mean(rollLog(idx)),  std(rollLog(idx)));
fprintf('  Pitch : mean = %.4f deg, std = %.4f deg\n', ...
    mean(pitchLog(idx)), std(pitchLog(idx)));
fprintf('  Yaw   : mean = %.4f deg, std = %.4f deg\n', ...
    mean(yawLog(idx)),   std(yawLog(idx)));

% calculate settled w_b as mean over last 30s
idxSettle = tLog > (tLog(end) - 30);
wbx_settled = mean(wbxLog(idxSettle));
wby_settled = mean(wbyLog(idxSettle));
wbz_settled = mean(wbzLog(idxSettle));

fprintf('\n  Settled gyro bias estimate (mean over last 30 s):\n');
fprintf('    w_bx = %.5f deg/s\n', wbx_settled);
fprintf('    w_by = %.5f deg/s\n', wby_settled);
fprintf('    w_bz = %.5f deg/s\n', wbz_settled);

%plot rpy
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

% plot w_b conv
figure('Color','w','Position',[100 100 1000 500]);
plot(tLog, wbxLog, tLog, wbyLog, tLog, wbzLog); grid on;
title('Estimated gyro bias (w\_b) convergence - Xsens MARG');
xlabel('time (s)'); ylabel('deg/s');
legend({'w\_bx','w\_by','w\_bz'}, 'Location','best');

save(outFile, 'tLog', 'rollLog', 'pitchLog', 'yawLog', ...
              'wbxLog', 'wbyLog', 'wbzLog', ...
              'wbx_settled', 'wby_settled', 'wbz_settled', 'duration');
fprintf('\nSaved: %s\n', outFile);