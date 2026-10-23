clear all
close all
clc
%% Simulating (sort of) Spotlight SAR with Stripmap SAR definitions
%% Parameters
R_eta_c     = 20e3;     % m    slant range of scene center
V_r         = 150;      % m/s
T_r         = 2.5e-6;   % s
K_r         = 20e12;    % Hz/s
f0          = 5.3e9;    % Hz
delta_f_dop = 80;       % Hz
F_r         = 60e6;     % Hz
F_a         = 100;      % Hz
N_az        = 256;
N_rg        = 320;
theta_r_c   = 0;      % deg , beam squint angle
eta_c       = -8.1;     % s , beam center crossing time
f_eta_c        = 320;      % Hz , doppler centroid freq, fixed at 2V_rsin(theta_r_c)/lambda for all ranges
% theta_r_c = 21.9; eta_c = -49.7; f_eta_c = 1975;
c      = 3e8;
lambda = c/f0;
theta_r_c  = deg2rad(theta_r_c);

%% Functions
function t = dist2time(d,mode,v)
    c = 3e8/2; % lightspeed
    switch nargin
        case 3
            c=v;
        case 2
        case 1
            mode = "time";
        otherwise
            error("Invalid outputs!")
    end

    t = d * ((mode=="time")/c + (mode=="dist")*c);
end

%% Scene definition

bin_delta_R = dist2time(1/F_r,"dist");
n_rg = 1:N_rg;
n_rg_centered = (n_rg-1-N_rg/2);
range_baseband = n_rg_centered*bin_delta_R;
range_centered = range_baseband + R_eta_c;
range_baseband_0 = range_baseband*cos(theta_r_c);
range_centered_0 = range_centered*cos(theta_r_c);
tau_baseband = dist2time(range_baseband);
tau_centered = dist2time(range_centered);

n_az = 1:N_az;
n_az_centered = (n_az-1-N_az/2);
eta_baseband = (n_az-1-N_az/2)/F_a;
eta_centered = eta_baseband + eta_c;
scene_delta_x = dist2time(1/F_r,"dist",V_r);
x_baseband = dist2time(eta_baseband,"dist",V_r);
x_centered = dist2time(eta_centered,"dist",V_r);
x_eta_c = dist2time(eta_c,"dist",V_r);

% [ETA, TAU] = meshgrid(eta_centered,tau_centered);
echoScene_BP = zeros(N_rg,N_az);



%% Chirp Definition (Rx)
R_tg_idx = N_rg/2;
% R_tg_idx = randi(N_rg);
eta_tg_idx = N_az/2;
% R_tg = R_eta_c*(1+0.005*cos(2*pi*n_az/N_az)); % doesn't work because we
% don't simulate the radar capturing everything, only capturing at the
% center of the beam
% R_tg = repmat(repmat(R_eta_c,1,N_az), 4,1) + repmat(1:4,N_az,1)';

R_tg = R_eta_c;
eta_tg = eta_c;

A0 = 1;
R0 = R_tg.*cos(theta_r_c);

for ii = 1:length(eta_centered) %captures
    K_a = 2*V_r^2*cos(theta_r_c)^2./(lambda*R_tg);
    eta = eta_centered(ii);

    R_eta = sqrt(R0^2 + (V_r^2) .* (eta_tg - eta_c)^2);
    
    w_r_shifted = abs(tau_centered - dist2time(R_eta)) <= T_r/2;
    w_a = sinc(0.886.*K_a.*(eta - eta_tg)/delta_f_dop).^2; %antenna

    echo = A0 .* w_r_shifted' .* w_a ...
        .* exp(-1j.*4.*pi.*f0.*R_eta./c) ...
        .* exp(1j.*pi.*K_r.*(tau_centered' - dist2time(R_eta)).^2);

    echoScene_BP(:,ii) = squeeze(echoScene_BP(:,ii)) + echo;
end

%% Plots
figure
subplot(1,2,1)
imagesc(real(echoScene_BP)')
view(2)
title("Real part")
subplot(1,2,2)
imagesc(20.*log10(abs(echoScene_BP)'./max(abs(echoScene_BP),[],"all")))
view(2)
title("Magnitude")

%% Range-Azimuth Compression
Beta_coeff = 2.5;

baseband_chirp = exp(1j.*pi.*K_r.*(tau_baseband').^2);
baseband_chirp = baseband_chirp((abs(tau_baseband) <= T_r/2));

nChirp = length(baseband_chirp);
win_kaiser = ifftshift(kaiser(N_rg,Beta_coeff));
MF = conj(fft(baseband_chirp,N_rg)).*win_kaiser;

s_rc_BP = ifft(fft(echoScene_BP,[],1).*MF,[],1);
s_rc_BP = s_rc_BP(1:end-nChirp+1,:);
Nr = size(s_rc_BP,1);

%% Plots
figure
subplot(1,2,1)
imagesc(real(squeeze(sum(s_rc_BP,2)))')
view(2)
title("Real part")
subplot(1,2,2)
imagesc(20.*log10(abs(squeeze(sum(s_rc_BP,2)))'./max(abs(squeeze(sum(s_rc_BP,2))),[],"all")))
view(2)
title("Magnitude")

%% BP

ramp_filter = triang(Nr);

ramp_threshold = 0.6;
ramp_peak = (1 + ramp_threshold)/2;

mask = ramp_filter > ramp_threshold;
uu = (ramp_filter(mask) - ramp_threshold)/(1-ramp_threshold);

ramp_filter(mask) = ramp_threshold + (ramp_peak-ramp_threshold)*(2*uu - uu.^2);

s_ed = zeros(Nr,N_az);
rot_angles = 360*n_az_centered/N_az;
for ii = 1:N_az
    s_ed_tau = ifft(fft(squeeze(s_rc_BP(:,ii))).*ramp_filter);
    s_ed_tau_rep = repmat(s_ed_tau,1,N_az)/N_az;
    s_ed = s_ed + imrotate(real(s_ed_tau_rep),rot_angles(ii),"bilinear","crop");
    s_ed = s_ed + 1j.*imrotate(imag(s_ed_tau_rep),rot_angles(ii),"bilinear","crop");
end
%%

figure
imagesc(20.*log10(abs(s_ed)'./max(abs(s_ed),[],"all")))
view(2)
title("Magnitude")



%% TO DO

% Learn about spotlight SAR and CSAR
% Fix the off center scanning

