%% Stripmap SAR back-projection of a measured X1000 capture
% Focuses an acquisition saved by radar_gui.py (.mat). The radar moves at
% constant speed along a straight rail (x axis) and looks along y, so a
% reflector at (x0, y0) is seen at range R = sqrt((x - x0)^2 + y0^2).
clear
close all
clc

%% Settings
data_file = "acquisition_20261117_113549.mat";  % in <repo>/Results
L_rail    = 0.46;             % m, rail travel over the recording
frames    = 400:4:6000;       % rail is still until frame ~140 and accelerating until ~400;
                              % every 4th frame (0.3 mm apart) is still finer than lambda/4
r_min     = 0.55;             % m, range window kept after range compression
r_max     = 1.0;              % m
x_step    = 0.5e-3;           % m, pixel size along track
y_coords  = 0.62:0.5e-3:0.86; % m, cross-track image axis

%% Load the capture
script_dir = fileparts(mfilename("fullpath"));
if isempty(script_dir)
    script_dir = pwd;         % Run Section / Evaluate Selection
end
% rx0_highpass is trace 0 (monostatic TRx) with the antenna coupling
% filtered out. rx_beamformed (complex) can be processed the same way, but
% its RX weights are calibrated at one angle, so its gain varies along
% the pass.
load(fullfile(script_dir, "..", "Results", data_file), "rx0_highpass", ...
    "frequency_hz", "trigger_timestamps_ns", "frame_timestamps_s");

%% Parameters from the capture
c   = 299792458;              % m/s, same value as radar_gui.py
f_n = frequency_hz(:);        % Hz, frequency of every IF sample of a sweep
N_s = numel(f_n);             % samples per sweep
df  = mean(diff(f_n));        % Hz, frequency step between samples
B   = f_n(end) - f_n(1);      % Hz, swept bandwidth
f0  = (f_n(1) + f_n(end))/2;  % Hz, centre frequency

%% Slow time and radar positions
% trigger_timestamps_ns is the radar clock; frame_timestamps_s is PC time
% with ~16 ms resolution (consecutive frames can share a time stamp).
if all(trigger_timestamps_ns >= 0)
    eta = double(trigger_timestamps_ns - trigger_timestamps_ns(1))*1e-9;  % s
else
    eta = frame_timestamps_s - frame_timestamps_s(1);                    % s
end
V_r       = L_rail/(eta(end) - eta(1));          % m/s, rail speed
pos_drone = V_r*(eta(frames) - eta(frames(1)));  % m, along-track radar position
N_az      = numel(frames);

%% Range compression
% Range profile = fft of the windowed sweep, as in radar_gui.py. Targets
% are in the positive half: bin k is at range k*c/(2*Nfft*df).
echoScene_BP = single(permute(rx0_highpass(frames,1,:), [3 1 2]));  % [N_s x N_az]
w_rg   = 0.5 - 0.5*cos(2*pi*(0:N_s-1).'/(N_s-1));  % Hann window
Nfft   = 2*2^nextpow2(N_s);                         % zero padding -> 0.8 mm bins
r_axis = (0:Nfft-1).'*c/(2*Nfft*df);               % m
keep   = r_axis >= r_min & r_axis <= r_max;
s_rc   = fft(echoScene_BP.*w_rg, Nfft, 1);
s_rc   = double(s_rc(keep,:))/sum(w_rg);
r_axis = r_axis(keep);
% fft references the phase to the first sample (f_n(1)); move the
% reference to the sweep centre so the back-projection can use f0.
s_rc   = s_rc.*exp(1j*2*pi*B*r_axis/c);

figure
imagesc(frames, r_axis, 20*log10(abs(s_rc)/max(abs(s_rc),[],"all")))
axis xy
clim([-40 0])
colorbar
xlabel("Frame")
ylabel("Range [m]")
title("Range-compressed data [dB]")

%% Image grid
x_coords = pos_drone(1):x_step:pos_drone(end);
[X,Y]    = ndgrid(x_coords, y_coords);
pixels   = X + 1j.*Y;

%% Back-projection
grid_mag = complex(zeros(size(pixels)));
for nn = 1:N_az
    R_pixel  = abs(pixels - pos_drone(nn));
    s_interp = interp1(r_axis, s_rc(:,nn), R_pixel, "linear", 0);
    grid_mag = grid_mag + s_interp.*exp(-1j*4*pi*f0*R_pixel/c);
end

%% Scene recreated
figure
imagesc(y_coords, x_coords, 20*log10(abs(grid_mag)/max(abs(grid_mag),[],"all")))
axis xy equal tight
colormap(gca, gray)
clim([-30 0])
colorbar
xlabel("Cross-track y [m]")
ylabel("Along-track x [m]")
title("Back-projected image [dB]")
