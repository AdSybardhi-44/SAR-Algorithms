clear all; close all; clc;
%%

fc = 5.405e9;
K = 20e12;
T  = 42e-6;
B = T*K;
% T_samples = 401;
alpha_sampling = 1.07;
fs = B*alpha_sampling;
% fs = 18.5e6;
% T = T_samples/fs;
c  = 3e8;

scene_samples = 2048;

% Two separate time vectors
t_chirp = 0:1/fs:T -1/fs;                % chirp duration
t_scene = 0:1e-4:499e-4;              % scene duration

% K = B/T;
%%

% Original chirp - chirp duration only
chirp_raw = exp(1j.*pi.*K.*t_chirp.^2);

% Target
R_target      = 30e-2;
tau           = 2*R_target/c;
delay_samples1 = ceil(tau*fs);
% delay_samples1 = 50;
% delay_samples2 = 800;
% delay_samples3 = 1600;

% Echo - place delayed chirp in scene
echo = zeros(1, length(t_scene));
echo(delay_samples1 : delay_samples1+length(chirp_raw)-1) = chirp_raw;
% echo(delay_samples2 : delay_samples2+length(chirp_raw)-1) = echo(delay_samples2 : delay_samples2+length(chirp_raw)-1) + chirp_raw;
% echo(delay_samples3 : delay_samples3+length(chirp_raw)-1) = echo(delay_samples3 : delay_samples3+length(chirp_raw)-1) + chirp_raw;

Nfft = scene_samples;
% MF from original chirp - zero padded to scene length
delta_K = K/1e2;
MF_uncalibrated = conj(fft(chirp_raw.*exp(1j.*pi.*delta_K.*t_chirp.^2), Nfft));
MF = conj(fft(chirp_raw , Nfft));

Beta_coeff = 2.5;
kaiser_win = ifftshift(kaiser(Nfft, Beta_coeff));

% Compress
compressed_uncal = ifft(fft(echo, Nfft) .* MF_uncalibrated .* kaiser_win');
compressed =  ifft(fft(echo, Nfft) .* MF .* kaiser_win');

% Range axis
range = (0:Nfft-1)*c/(2*fs);

figure
plot(range/1e3, abs(compressed_uncal)/max(abs(compressed_uncal)))
hold on
plot(range/1e3, abs(compressed)/max(abs(compressed)))
hold off
xlabel('Range (km)')
ylabel('Normalized Amplitude')
title('Single Target')
axis tight

% range_uncorrupted = range(1:end-T_samples+1);
% compressed_uncorrupted = compressed_uncal(1:end-T_samples+1);
% figure
% plot(range_uncorrupted/1e3, abs(compressed_uncorrupted)/max(abs(compressed_uncorrupted)) )
% xlabel('Range (km)')
% ylabel('Normalized Amplitude')
% title('Single Target (uncorrupted)')
% axis tight
