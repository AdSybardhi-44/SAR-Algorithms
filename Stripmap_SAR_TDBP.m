clear all
close all
clc
%% Load radar data
load("radar_data.mat")
load("range_offset.mat","internal_offset")

%% Load actuator data
actuator_csv_to_mat("actuator_data.csv", "actuator_data.mat", radar_timestamps)
load("actuator_data.mat")                                                   % Stage positions (actuator_csv_to_mat.m)

%% Extract radar parameters
L_scene_measured = 0.5;                                                     % Measured scene length [m]
R_furthest_scene = 0.6;                                                     % Furthest scene range [m]

lambda = radar_carrier_wavelength;                                          % Radar carrier wavelength [m]
fc = radar_carrier_freq;                                                    % Radar carrier frequency [Hz]
B = radar_bandwidth;                                                        % Radar bandwidth [Hz]
T_r = radar_sweep_time;                                                     % Radar sweep time [s]
gamma = B/T_r;                                                                % Chirp rate [Hz/s]
c = radar_lightspeed_coefficient;                                           % Speed of light [m/s]
PRF = 1/radar_sweep_period;                                                 % Pulse repetition frequency [Hz]
Fr = 1/radar_sample_interval;                                               % Sampling frequency [Hz]
N_samples = radar_range_samples_number;                                     % Number of radar samples
sampling_time_axis = (0:N_samples-1)*radar_sample_interval;                 % Sampling time axis [s]
res_rg = radar_range_resolution;                                            % Range resolution [m]
res_az = lambda/(4*sin(atan(L_scene_measured/(2*R_furthest_scene))));       % Azimuth resolution (theoretically) [m]
res_az_max = lambda/4;
radar_mag_norm = radar_full_scale;                                          % Radar IF magnitude normalization factor
N_rx = radar_rx_number;                                                     % Number of radar receivers
f_tau = radar_frequency_axis;                                               % Radar range frequency axis [Hz]

%% Radar data
range_oversampling = 16;                                                    % Range oversampling factor
Nfft = 2^(nextpow2(N_samples)) * range_oversampling;
df = abs(f_tau(2) - f_tau(1));
delta_R = c / (2 * Nfft * df);                                              % Range bin [m]
N_rg = 1 + Nfft/2;
echoScene_all_rx = double(radar_signal) ./ radar_mag_norm;                  % Extracted radar echo scene [real-valued] (N_sam x N_rx x N_frames)
echoScene_all_rx = echoScene_all_rx - mean(echoScene_all_rx,1);
echoScene = squeeze(echoScene_all_rx(:,1,:));                               % Extracted radar echo scene for TRx [real-valued] (N_sam x N_frames)
range_centered = ((0:N_rg-1)'*delta_R) - internal_offset;                   % Centered radar range samples starting from APC [m] (N_rg x 1)

%% Actuator data
x_act = actuator_position;                                                  % Stage position along the rail [m]
t_act = actuator_timestamps;                                                % Stage timestamps [s]
timestamp_mask = t_act(1) <= radar_timestamps & radar_timestamps <= t_act(end);
radar_timestamps = radar_timestamps(timestamp_mask);                        % Radar slow time axis [s]
eta = radar_timestamps - radar_timestamps(1);                               % Radar slow time axis [s] (1 x N_az)
x_radar = interp1(t_act, x_act, radar_timestamps, "linear");                % Interpolated radar position at radar timestamps [m] (1 x N_az)

% Possible downsampling in the azimuth direction due to the high azimuth sampling rate (to be defined later if needed)
N_az = length(radar_timestamps);                                            % Number of azimuth samples
n_az = (1:N_az);

%% Scene setup (acquired and pixels)
R_max = range_centered(end);                                                % Maximum range [m]
R_scene_center = 0.5;                                                       % Center of the scene range [m]
swath_width = 0.1;                                                          % Swath width [m]
swath_mask = range_centered >= R_scene_center - swath_width/2 & ...
    range_centered <= R_scene_center + swath_width/2;
swath = range_centered(swath_mask);                                         % Range samples within the swath [m] (N_sw x 1)
N_sw = length(swath);                                                       % Number of range samples within the swath [m]
n_sw = (1:N_sw)';                                                           % Range sample indices (N_sw x 1)

radar_position = x_radar + 1j*0;                                            % Radar position at radar timestamps [m]

y_oversampling = 2;
delta_y = res_rg/y_oversampling;                                            % Range spacing [m]
y_max = max(swath);                                                         % Maximum range position [m]
y_min = min(swath);                                                         % Minimum range position [m]

y_coords = y_min:delta_y:y_max;                                             % Range pixel positions [m] (1 x N_y)
N_y = length(y_coords);
y_coords_idx = 1:N_y;

x_oversampling = 2;                                                         % Cross-range oversampling factor
delta_x = res_az_max/x_oversampling;                                        % Cross-range spacing [m]
x_max = max(x_radar);                                                       % Maximum cross-range position [m]
x_min = min(x_radar);                                                       % Minimum cross-range position [m]
% x_max = max_value_chosen;                                                       % Maximum cross-range position [m]
% x_min = min_value_chosen;                                                       % Minimum cross-range position [m]

x_coords = x_min:delta_x:x_max;                                             % Cross-range pixel positions [m] (1 x N_x)
N_x = length(x_coords);
x_coords_idx = 1:N_x;

[X, Y] = ndgrid(x_coords, y_coords);                                        % 2D grid of cross-range and range pixel positions [m]
Pixels = X + 1j*Y;                                                          % Complex representation of pixel positions [m] (N_x x N_y)
% Pixel representation needs to be fixed, so we can represent the recreated
% image in any plane we want

% radar2pixel = Pixels(:) - radar_position;                                   % Vector from radar to each pixel
% radar2pixel_range = abs(radar2pixel);                                       % Magnitude of range from radar to each pixel [m]
% radar2pixel_angle = angle(radar2pixel);                                     % Phase of range from radar to each pixel [rad]

%% Processing

%% Processing the sampled Echo of the Scene
echoScene = echoScene(:, timestamp_mask);                                   % Extract the echo corresponding to the non-idle frames
% Possible downsampling in the azimuth direction (to be defined later if needed)

fig_unprocessed = figure(1);
imagesc(sampling_time_axis, eta, echoScene.');
axis xy;
xlabel('ADC sampling time [s]');
ylabel('Slow time [s]');
title("Unprocessed Echo of the full Scene")

%% Range Compression
s_rc_full = ifft(echoScene, Nfft, 1) .* ...
    exp(-1j*(pi*(0:Nfft-1)'/Nfft)*(N_samples -1));                          % Perform zero-phase ramp range compression using inverse FFT
s_rc_full_unique = s_rc_full(1:length(range_centered), :);                  % Keep only the first unique N_rg range samples

fig_range_compressed_full = figure(2);
subplot(2, 2, 1)
imagesc(range_centered, eta, real(s_rc_full_unique).');
axis xy;
xlabel('Range [m]');
ylabel('Slow time [s]');
title('Real Part');
subplot(2, 2, 2)
imagesc(range_centered, eta, imag(s_rc_full_unique).');
axis xy;
xlabel('Range [m]');
ylabel('Slow time [s]');
title('Imaginary Part');
subplot(2, 2, 3)
imagesc(range_centered, eta, 20*log10(abs(s_rc_full_unique)/max(abs(s_rc_full_unique(:)))).');
axis xy;
xlabel('Range [m]');
ylabel('Slow time [s]');
title('Magnitude [dB]');
clim([-40 0]);
subplot(2, 2, 4)
imagesc(range_centered, eta, angle(s_rc_full_unique).');
axis xy;
xlabel('Range [m]');
ylabel('Slow time [s]');
title('Phase');
sgtitle('Range Compressed Echo of the full Scene');

%% Focusing on the swath

s_rc = s_rc_full_unique(swath_mask, :);                                     % Extract the range-compressed echo corresponding to the swath width

fig_range_compressed_swath = figure(3);
subplot(2, 2, 1)
imagesc(swath, eta, real(s_rc).');
axis xy;
xlabel('Range [m]');
ylabel('Slow time [s]');
title('Real Part');
subplot(2, 2, 2)
imagesc(swath, eta, imag(s_rc).');
axis xy;
xlabel('Range [m]');
ylabel('Slow time [s]');
title('Imaginary Part');
subplot(2, 2, 3)
imagesc(swath, eta, 20*log10(abs(s_rc)/max(abs(s_rc(:)))).');
axis xy;
xlabel('Range [m]');
ylabel('Slow time [s]');
title('Magnitude [dB]');
clim([-40 0]);
subplot(2, 2, 4)
imagesc(swath, eta, angle(s_rc).');
axis xy;
xlabel('Range [m]');
ylabel('Slow time [s]');
title('Phase');
sgtitle('Range Compressed Echo of the Swath');

%% Backprojection Algorithm

grid_mag = complex(zeros(N_x, N_y));
for nn = 1:N_az
    rel_pixel = Pixels - radar_position(nn);
    R_pixel = abs(rel_pixel);

    s_interp = interp1(swath, s_rc(:,nn),R_pixel, "linear",0);
    phi = fc*2*R_pixel/c - 0.5*gamma*(2*R_pixel/c).^2; % if IF was used
    grid_mag = grid_mag + s_interp.*exp(1j*2*pi*phi);
end