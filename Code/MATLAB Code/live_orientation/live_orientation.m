%  Reads quaternion from Arduino (could be any filter as long as it returns quaternion) and plots live 3D orientation.
clear; clc; close all;

% Connect to Arduino
delete(serialportfind);
s = serialport("COM3", 115200); % Change to correct port 
s.Timeout = 6;
configureTerminator(s, "LF");
pause(2);
flush(s);

viewer = HelperOrientationViewer;

while true
    try
        line = readline(s);
    catch
        continue
    end

    if strlength(strtrim(line)) == 0
        continue
    end

    values = str2double(strsplit(line, ','));
    if length(values) ~= 4 || any(isnan(values))
        continue;
    end

    q = quaternion(values(1), values(2), values(3), values(4));
    % Apply 180° rotation about x axis to convert sensor frame (Z-up) to NED viewer frame (Z-down).
    % The displayed roll, pitch, and yaw correspond to rotations about the sensor's
    % own x, y, and z axes respectively — not aligned to any absolute reference direction.
    q_align = quaternion(0, 1, 0, 0);
    viewer(q_align * q);
end
