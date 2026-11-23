clear all
close all
clc
%% load data
load("radar_data.mat")
%% radar parameters and data
fc = radar_carrier_freq;
lambda = radar_carrier_wavelength;
B = radar_bandwidth;
T_r = radar_sweep_time;
K_r = B/T_r;
c = radar_lightspeed_coefficient;
V_r = calculate_velocity(radar_accelerometer_data);
N_rg = radar_range_samples_number/2;
N_az = radar_frames;
echoScene = zeros(N_rg,N_az,radar_rx_number);
for rx = 1:radar_rx_number
    echoScene(:,:,rx) = squeeze(radar_signal(rx));
end
res_rg = radar_range_resolution;
res_az = radar_azimuth_resolution;

range_centered = radar_range_samples(first_half_or_until_max_range_needed);
eta_centered = radar_timestamps - radar_timestamps(1);

n_rg = 1:N_rg;
n_az = 1:N_az;
other_important_variables;
%% scene setup