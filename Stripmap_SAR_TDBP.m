clear all
close all
clc
%% Load radar data
load("radar_data.mat")
% load("range_offset.mat")

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
K_r = B/T_r;                                                                % Chirp rate [Hz/s]
c = radar_lightspeed_coefficient;                                           % Speed of light [m/s]
PRF = 1/radar_sweep_period;                                                 % Pulse repetition frequency [Hz]
Fr = 1/radar_sample_interval;                                               % Sampling frequency [Hz]
sampling_time_axis = (0:N_samples-1)*radar_sample_interval;                 % Sampling time axis [s]
res_rg = radar_range_resolution;                                            % Range resolution [m]
res_az = lambda/(4*sin(atan(L_scene_measured/(2*R_furthest_scene))/2));     % Azimuth resolution (theoretically) [m]
radar_mag_norm = radar_full_scale;                                          % Radar IF magnitude normalization factor
N_rx = radar_rx_number;                                                     % Number of radar receivers
N_samples = radar_range_samples_number;                                     % Number of radar samples
N_rg = floor(radar_range_samples_number/2) + 1;                             % Number of range samples
N_az = radar_frames;                                                        % Number of azimuth samples

%% Radar data
echoScene = radar_signal;                                                   % Extracted radar echo scene [real-valued]
f_tau = radar_frequency_axis;                                               % Radar range frequency axis [Hz]
range_centered = radar_range_samples(1:N_rg);                               % Centered radar range samples [m]

%% Actuator data
x_act = actuator_position;                                                  % Stage position along the rail [m]
t_act = actuator_timestamps;                                                % Stage timestamps [s]
radar_timestamps = radar_timestamps(...
    t_act(1) <= radar_timestamps & radar_timestamps <= t_act(end));         % Radar slow time axis [s]
eta = radar_timestamps - radar_timestamps(1);                               % Radar slow time axis [s]
x_radar = interp1(t_act, x_act, radar_timestamps, "linear");                % Interpolated radar position at radar timestamps [m]

% Possible downsampling in the azimuth direction due to the high azimuth sampling rate (to be defined later if needed)
N_az = length(radar_timestamps);                                            % Number of azimuth samples
n_az = (0:N_az-1)';                                                         % Azimuth sample indices '

%% Scene setup (acquired and pixels)
R_max = range_centered(end);                                                % Maximum range [m]
R_scene_center = 0.5;                                                       % Center of the scene range [m]
swath_width = 0.1;                                                          % Swath width [m]
swath = range_centered(range_centered >= R_scene_center - swath_width/2 & ...
    range_centered <= R_scene_center + swath_width/2);                      % Range samples within the swath [m]
N_rg = length(swath);                                                       % Number of range samples within the swath [m]
n_rg = (0:N_rg-1)';                                                         % Range sample indices '
% swath = swath - internal_offset;                                            % Calibrated swath range [m]

radar_position = x_radar + 1j*0;                                            % Radar position at radar timestamps [m]

azimuth_oversampling = 4;                                                   % Azimuth oversampling factor
delta_x = res_az/4;                                                         % Cross-range spacing [m]
x_max = max(x_radar);                                                       % Maximum cross-range position [m]
x_min = min(x_radar);                                                       % Minimum cross-range position [m]
% x_max = max_value_chosen;                                                       % Maximum cross-range position [m]
% x_min = min_value_chosen;                                                       % Minimum cross-range position [m]

x_coords = x_min:delta_x:x_max;                                             % Cross-range pixel positions [m]
x_coords_idx = 1:length(x_coords);

range_oversampling = 4;                                                     % Range oversampling factor
delta_y = res_rg/4;                                                         % Range spacing [m]
y_max = max(swath);                                                         % Maximum range position [m]
y_min = min(swath);                                                         % Minimum range position [m]

y_coords = y_min:delta_y:y_max;                                             % Range pixel positions [m]
y_coords_idx = 1:length(y_coords);

[X, Y] = ndgrid(x_coords, y_coords);                                        % 2D grid of cross-range and range pixel positions [m]
Pixels = X + 1j*Y;                                                          % Complex representation of pixel positions [m]

radar2pixel = Pixels - radar_position;                                      % Range from radar to each pixel [m]
range2pixel_range = abs(radar2pixel);                                       % Magnitude of range from radar to each pixel [m]
range2pixel_angle = angle(radar2pixel);                                     % Phase of range from radar to each pixel [rad]

%% Post-processing

%% Processing the sampled Echo of the Scene
echoScene = echoScene(:, ...
    t_act(1) <= radar_timestamps & radar_timestamps <= t_act(end));         % Extract the echo corresponding to the non-idle frames
% Possible downsampling in the azimuth direction (to be defined later if needed)

fig_unprocessed = figure(1);
imagesc(eta, sampling_time_axis, transpose(echoScene));
xlabel('Slow time [s]');
ylabel('Fast time [s]');
title('Unprocessed Echo of the full Scene');

%% Range Compression
s_rc = ifft(echoScene, [], 1);  % Perform range compression using inverse FFT
s_rc = s_rc(1:N_rg, :);                                                     % Keep only the first unique N_rg range samples

fig_range_compressed_full = figure(2);
subplot(2, 2, 1)
imagesc(eta, range_centered, transpose(real(s_rc)));
xlabel('Slow time [s]');
ylabel('Range [m]');
title('Real Part');
subplot(2, 2, 2)
imagesc(eta, range_centered, transpose(imag(s_rc)));
xlabel('Slow time [s]');
ylabel('Range [m]');
title('Imaginary Part');
subplot(2, 2, 3)
imagesc(eta, range_centered, transpose(20*log10(abs(s_rc)/max(abs(s_rc(:))))));
xlabel('Slow time [s]');
ylabel('Range [m]');
title('Magnitude [dB]');
caxis([-40 0]);  % Set color axis for better visualization of magnitude in dB
subplot(2, 2, 4)
imagesc(eta, range_centered, transpose(angle(s_rc)));
xlabel('Slow time [s]');
ylabel('Range [m]');
title('Phase');
sgtitle('Range Compressed Echo of the full Scene');

%% Focusing on the swath

s_rc = s_rc(range_centered >= R_scene_center - swath_width/2 & ...
    range_centered <= R_scene_center + swath_width/2, :);                   % Extract the range-compressed echo corresponding to the swath width

fig_range_compressed_swath = figure(3);
subplot(2, 2, 1)
imagesc(eta, swath, transpose(real(s_rc)));
xlabel('Slow time [s]');
ylabel('Range [m]');
title('Real Part');
subplot(2, 2, 2)
imagesc(eta, swath, transpose(imag(s_rc)));
xlabel('Slow time [s]');
ylabel('Range [m]');
title('Imaginary Part');
subplot(2, 2, 3)
imagesc(eta, swath, transpose(20*log10(abs(s_rc)/max(abs(s_rc(:))))));
xlabel('Slow time [s]');
ylabel('Range [m]');
title('Magnitude [dB]');
caxis([-40 0]);
subplot(2, 2, 4)
imagesc(eta, swath, transpose(angle(s_rc)));
xlabel('Slow time [s]');
ylabel('Range [m]');
title('Phase');
sgtitle('Range Compressed Echo of the Swath');

%% Backprojection Algorithm

grid_mag = zeros(N_x, N_y);
for xx = x_coords_idx
    for yy = y_coords_idx
        tg_pixel = Pixels(xx,yy);
        rel_pixel = tg_pixel - radar_position;
        R_pixel = abs(rel_pixel);
        N_INTERP = (R_pixel - swath(1))/delta_R +1;
        N_SAM = n_rg;
        for nn = n_az
            INTERP_VALUES = N_INTERP(nn) - N_SAM;
            interp_neighbors = 16;
            mask_interp = abs(INTERP_VALUES)<=interp_neighbors/2;
            INTERP_KERNEL = zeros(size(INTERP_VALUES));
            INTERP_WINDOW = hamming(interp_neighbors);
            INTERP_KERNEL(mask_interp) = sinc(INTERP_VALUES(mask_interp)).*(0.54 +0.46*cos(2*pi*INTERP_VALUES(mask_interp)/interp_neighbors));
            echoScene2Pixel = INTERP_KERNEL*(s_rc(:,nn))*exp(1j.*4.*pi.*f0.*R_pixel(nn)./c);

            grid_mag(xx,yy) = grid_mag(xx,yy) + echoScene2Pixel;
        end
    end
end