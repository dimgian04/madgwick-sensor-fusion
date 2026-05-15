clear;
clc;
close all;

%  For Xsens MARG filter (mti3_madgwick_marg.ino with w_bx,w_by,w_bz also printed)
port     = "COM3";
baud     = 115200;
duration = 60;
outFile  = 'beta-0.06_zeta-0.005.mat'; % make sure to change this to the correct values so u dont overwrite old logs

delete(serialportfind);
s = serialport(port, baud);
configureTerminator(s, "LF");
s.Timeout = 10;
pause(5);
flush(s);

fprintf('--- Logging XSENS_MARG for %d seconds ---\n', duration);
fprintf('Keep sensor STATIONARY.\n\n');

% LIVE PLOT SETUP
% RPY figure
figRPY = figure('Color','w','Position',[100 100 1000 700]);

ax1 = subplot(3,1,1);
alRoll = animatedline(ax1,'Color','b'); grid on;
title('Roll'); ylabel('deg'); xlim([0 duration]); %ylim([-180 180]); 

ax2 = subplot(3,1,2);
alPitch = animatedline(ax2,'Color','r'); grid on;
title('Pitch'); ylabel('deg'); xlim([0 duration]);%ylim([-180 180]); 

ax3 = subplot(3,1,3);
alYaw = animatedline(ax3,'Color','g'); grid on;
title('Yaw'); ylabel('deg'); xlabel('time (s)'); xlim([0 duration]);%ylim([-180 180]); 

% w_b figure
figWb = figure('Color','w','Position',[100 800 1000 500]);
axWb = gca;
alWbx = animatedline(axWb,'Color','b','DisplayName','w\_bx');
alWby = animatedline(axWb,'Color','r','DisplayName','w\_by');
alWbz = animatedline(axWb,'Color','g','DisplayName','w\_bz');
grid on;
title('Estimated gyro bias (w\_b) convergence - Xsens MARG');
xlabel('time (s)'); ylabel('deg/s'); xlim([0 duration]);
legend('Location','best');

% Logs
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
        t_now     = toc(t0);
        roll_deg  = rad2deg(roll);
        pitch_deg = rad2deg(pitch);
        yaw_deg   = rad2deg(yaw);

        tLog(k,1)     = t_now;
        rollLog(k,1)  = roll_deg;
        pitchLog(k,1) = pitch_deg;
        yawLog(k,1)   = yaw_deg;
        wbxLog(k,1)   = w_b(1);
        wbyLog(k,1)   = w_b(2);
        wbzLog(k,1)   = w_b(3);

        % update live plots every 10 samples
        if mod(k, 10) == 0
            addpoints(alRoll,  t_now, roll_deg);
            addpoints(alPitch, t_now, pitch_deg);
            addpoints(alYaw,   t_now, yaw_deg);
            addpoints(alWbx,   t_now, w_b(1));
            addpoints(alWby,   t_now, w_b(2));
            addpoints(alWbz,   t_now, w_b(3));
            drawnow limitrate;
        end

    catch
        continue;
    end
end

clear s;
delete(serialportfind);


idx = tLog > 30; % stats after 30 seconds

fprintf('\n===== XSENS_MARG =====\n');
fprintf('  Roll  : mean = %.4f deg, σ = %.4f deg, 3σ = %.4f\n', mean(rollLog(idx)),  std(rollLog(idx)), 3*std(rollLog(idx)));
fprintf('  Pitch : mean = %.4f deg, σ = %.4f deg, 3σ = %.4f\n', mean(pitchLog(idx)), std(pitchLog(idx)), 3*std(pitchLog(idx)));
fprintf('  Yaw   : mean = %.4f deg, σ = %.4f deg, 3σ = %.4f\n', mean(yawLog(idx)),   std(yawLog(idx)), 3*std(yawLog(idx)));

% calculate settled w_b as mean over last 5s
idxSettle = tLog > (tLog(end) - 5);
wbx_settled = mean(wbxLog(idxSettle));
wby_settled = mean(wbyLog(idxSettle));
wbz_settled = mean(wbzLog(idxSettle));

fprintf('\n  Settled gyro bias estimate (mean over last 5 s):\n');
fprintf('    w_bx = %.5f deg/s\n', wbx_settled);
fprintf('    w_by = %.5f deg/s\n', wby_settled);
fprintf('    w_bz = %.5f deg/s\n', wbz_settled);

%% ===== SAVE =====
save(outFile, 'tLog', 'rollLog', 'pitchLog', 'yawLog', ...
              'wbxLog', 'wbyLog', 'wbzLog', ...
              'wbx_settled', 'wby_settled', 'wbz_settled', 'duration');
fprintf('\nSaved: %s\n', outFile);