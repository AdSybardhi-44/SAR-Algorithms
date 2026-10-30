clear all
close all
clc
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
theta_r_c   = 3.5;      % deg , beam squint angle
eta_c       = -8.1;     % s , beam center crossing time
f_eta_c        = 320;      % Hz , doppler centroid freq, fixed at 2V_rsin(theta_r_c)/lambda for all ranges
% theta_r_c = 21.9; eta_c = -49.7; f_nc = 1975;
c      = 3e8;
lambda = c/f0;
theta_r_c  = deg2rad(theta_r_c);

%% Scene definition
scene_delta_R = c/(2*F_r);
n_rg = 1:N_rg;
range_centered = (n_rg-1-N_rg/2)*scene_delta_R + R_eta_c;
tau_centered = 2*range_centered/c;

n_az = 1:N_az;
eta_centered = (n_az-1-N_az/2)/F_a + eta_c;
scene_delta_x = V_r/F_a;
x_centered = eta_centered*V_r;

[ETA, TAU] = meshgrid(eta_centered,tau_centered);
echoScene = zeros(size(TAU));
%% Chirp definition (received)
target_rg_idx = [145 145 160];
target_az_idx = [90 110 110];   % A earlier than B and C; tune until the knot is near 175
targets_R   = range_centered(target_rg_idx);
targets_eta = eta_centered(target_az_idx);   % beam-center crossing times

nTargets = length(target_rg_idx);

A0 = 1;

for ii = 1:nTargets
    R_tg = targets_R(ii);
    R0 = R_tg.*cos(theta_r_c);
    eta_tg = targets_eta(ii);
    eta_0 = eta_tg - eta_c;                      % zero-Doppler time of this target

    K_a = 2*V_r^2*cos(theta_r_c)^2/(lambda*R_tg);
    w_a = sinc(0.886.*K_a.*(ETA - eta_c)./delta_f_dop).^2;

    R_eta = sqrt(R0^2 + V_r^2*(ETA - eta_0).^2);

    w_r_shifted = abs(TAU - 2*R_eta/c) <= T_r/2;

    echo = A0 .* w_r_shifted .* w_a ...
        .* exp(-1j.*4.*pi.*f0.*R_eta./c) ...
        .* exp(1j.*pi.*K_r.*(TAU - 2.*R_eta./c).^2);
    echoScene = echoScene + echo;
end

%% Initial Plots

figure
subplot(2,2,1)
imagesc(real(echoScene)')
view(2)
title("Real part")
subplot(2,2,2)
imagesc(imag(echoScene)')
view(2)
title("Imaginary part")
subplot(2,2,3)
imagesc(20.*log10(abs(echoScene)'./max(abs(echoScene),[],"all")))
view(2)
title("Magnitude")
subplot(2,2,4)
imagesc(angle(echoScene)')
view(2)
title("Phase")

%% Range compression
Beta_coeff = 2.5;
tau_baseband = tau_centered' - 2*R_eta_c/c;
baseband_chirp = exp(1j.*pi.*K_r.*(tau_baseband).^2);
baseband_chirp = baseband_chirp((abs(tau_baseband) <= T_r/2));
nChirp = length(baseband_chirp);
win_kaiser = ifftshift(kaiser(N_rg,Beta_coeff));
MF = conj(fft(baseband_chirp,N_rg)).*win_kaiser;

s_rc = ifft(fft(echoScene,[],1).*MF,[],1);
s_rc = s_rc(1:end-nChirp+1,:);
% s_rc = s_rc(40:40+100-1,:); %to match the book

%% Compression Plots

figure
subplot(1,2,1)
imagesc(real(s_rc)')
view(2)
title("Real part")
subplot(1,2,2)
imagesc(20.*log10(abs(s_rc)'./max(abs(s_rc),[],"all")))
view(2)
title("Magnitude")

%% Theoretical Compression
echoScene_theo = zeros(size(TAU));
for ii = 1:length(target_rg_idx)
    R_tg = targets_R(ii);
    R0 = R_tg.*cos(theta_r_c);
    eta_tg = targets_eta(ii);
    eta_0 = eta_tg - eta_c;                      % zero-Doppler time of this target

    K_a = 2*V_r^2*cos(theta_r_c)^2/(lambda*R_tg);
    w_a = sinc(0.886.*K_a.*(ETA - eta_tg)./delta_f_dop).^2;

    R_eta = sqrt(R0^2 + V_r^2*(ETA - eta_0).^2);

    B_r     = abs(K_r)*T_r;
    gamma_w = 1.18;             % Kaiser, beta = 2.5
    
    p_r = sinc(B_r .* (TAU - 2*R_eta./c) ./ gamma_w);

    echo = A0 .* p_r .* w_a ...
        .* exp(-1j.*4.*pi.*f0.*R_eta./c);
    echoScene_theo = echoScene_theo + echo;
end

s_rc_theo = echoScene_theo;
% s_rc_theo = echoScene_theo(150-36:150-1+100-36, :); % to match book

figure
subplot(1,2,1)
imagesc(real(s_rc_theo)')
view(2)
title("Real part")
subplot(1,2,2)
imagesc(20.*log10(abs(s_rc_theo)'./max(abs(s_rc_theo),[],"all")))
view(2)
title("Magnitude")


%% Azimuth FFT
S1 = fft(s_rc,[],2);
S1_theo = fft(s_rc_theo,[],2);

figure
subplot(2,2,1)
imagesc(real(S1)')
view(2)
title("Real part")
subplot(2,2,2)
imagesc(20.*log10(abs(S1)'./max(abs(S1),[],"all")))
view(2)
title("Magnitude")
subplot(2,2,3)
imagesc(real(S1_theo)')
view(2)
title("Real part")
subplot(2,2,4)
imagesc(20.*log10(abs(S1_theo)'./max(abs(S1_theo),[],"all")))
view(2)
title("Magnitude")

%% RCMC Sinc Interp
f_base = [0:N_az/2-1, -N_az/2:-1]*F_a/N_az; % fft ordering
f_eta = f_eta_c + mod(f_base - f_eta_c + F_a/2, F_a) - F_a/2; % unwrapped around centroid

Nr = size(S1,1);
R0 = (range_centered(1:Nr).'*cos(theta_r_c));          % Nr x 1
delta_R = lambda^2 * R0 .* (f_eta.^2) / (8*V_r^2);          % Nr x N_az
% delta_R = delta_R_centered - lambda^2 * R0 .* f_eta_c^2 / (8*V_r^2);


delta_R_idx = delta_R / scene_delta_R;
S2 = zeros(size(S1));

for jj = 1:N_az
    interp_len = 4;
    n_sam = 1:Nr;
    S1_sam = S1(:,jj);
    n_interp = 1:1/interp_len:Nr;
    [N_INTERP,N_SAM] = ndgrid(n_interp,n_sam);

    S1_interp = sinc(N_INTERP - N_SAM)*S1_sam;

    i = round((n_sam' + delta_R_idx(:,jj) - 1)*interp_len) + 1;
    bad = i < 1 | i > length(n_interp);
    i(bad) = 1;

    S2(:,jj) = S1_interp(i);
    S2(bad,jj) = 0;
end

figure
subplot(1,2,1)
imagesc(real(S2)')
view(2)
title("Real part")
subplot(1,2,2)
imagesc(20.*log10(abs(S2)'./max(abs(S2),[],"all")))
view(2)
title("Magnitude")

%% RCMC Phase multiplier
f_tau = ifftshift(-floor(Nr/2):ceil(Nr/2)-1).' * F_r/Nr;
delta_R_invariant = lambda^2 * R_eta_c * cos(theta_r_c) * f_eta.^2 / (8*V_r^2);
G_rcmc = exp(1j.*4.*pi.*f_tau.*delta_R_invariant./c);

S2_PM = ifft(fft(S1,[],1).*G_rcmc,[],1);
figure
subplot(1,2,1)
imagesc(real(S2_PM)')
view(2)
title("Real part")
subplot(1,2,2)
imagesc(20.*log10(abs(S2_PM)'./max(abs(S2_PM),[],"all")))
view(2)
title("Magnitude")

%% Azimuth Compresison
K_a = 2*V_r^2*cos(theta_r_c)^2./(lambda*range_centered(1:size(S2,1))); % same R0 from last section
H_az = exp(-1j.*pi.*(f_eta.^2)./K_a');
Beta_coeff = 2.5;
x = 2*(f_eta - f_eta_c)/F_a;                                   % -1..1 across the unwrapped band
win_az = besseli(0, Beta_coeff*sqrt(max(1-x.^2,0))) / besseli(0,Beta_coeff);   % 1 x N_az
S3 = S2 .* (H_az .* win_az);

T_a = delta_f_dop*R_eta_c*lambda/(2*V_r^2);
nAzimuth = round(T_a*F_a);
nThrowaway = (nAzimuth-1)/2;

s_ac = ifft(S3,[],2);
s_ac = s_ac(:,1+nThrowaway:end-nThrowaway);

peaks_tf = islocalmax2(abs(s_ac),MaxNumExtrema=3);
[range_coord, azimuth_coord] = meshgrid(1:size(s_ac,1),1:size(s_ac,2));

range_coord = range_coord';
azimuth_coord = azimuth_coord';


%%
figure
surf(20.*log10(abs(s_ac)'./max(abs(s_ac),[],"all")))
hold on
plot3(range_coord(peaks_tf),azimuth_coord(peaks_tf),abs(s_ac(peaks_tf)),"red.",MarkerSize=12)
% view(2)
title("Magnitude")

%% SRC Option 2
theta_r_c = 21.9; eta_c = -49.7; f_eta_c = 1975;
theta_r_c  = deg2rad(theta_r_c);

%% Scene definition


scene_delta_R = c/(2*F_r);
n_rg = 1:N_rg;
range_centered = (n_rg-1-N_rg/2)*scene_delta_R + R_eta_c;
tau_centered = 2*range_centered/c;

n_az = 1:N_az;
eta_centered = (n_az-1-N_az/2)/F_a + eta_c;
scene_delta_x = V_r/F_a;
x_centered = eta_centered*V_r;

[ETA, TAU] = meshgrid(eta_centered,tau_centered);
echoScene = zeros(size(TAU));
%% Chirp definition (received)
target_rg_idx = [155 155 175];
target_az_idx = [85 115 115];   % A earlier than B and C; tune until the knot is near 175
targets_R   = range_centered(target_rg_idx);
targets_eta = eta_centered(target_az_idx);   % beam-center crossing times

nTargets = length(target_rg_idx);

A0 = 1;

for ii = 1:nTargets
    R_tg = targets_R(ii);
    R0 = R_tg.*cos(theta_r_c);
    eta_tg = targets_eta(ii);
    eta_0 = eta_tg - eta_c;                      % zero-Doppler time of this target

    K_a = 2*V_r^2*cos(theta_r_c)^2/(lambda*R_tg);
    w_a = sinc(0.886.*K_a.*(ETA - eta_tg)./delta_f_dop).^2;

    R_eta = sqrt(R0^2 + V_r^2*(ETA - eta_0).^2);

    w_r_shifted = abs(TAU - 2*R_eta/c) <= T_r/2;

    echo = A0 .* w_r_shifted .* w_a ...
        .* exp(-1j.*4.*pi.*f0.*R_eta./c) ...
        .* exp(1j.*pi.*K_r.*(TAU - 2.*R_eta./c).^2);
    echoScene = echoScene + echo;
end

%% Initial Plots

figure
subplot(2,2,1)
imagesc(real(echoScene)')
view(2)
title("Real part")
subplot(2,2,2)
imagesc(imag(echoScene)')
view(2)
title("Imaginary part")
subplot(2,2,3)
imagesc(20.*log10(abs(echoScene)'./max(abs(echoScene),[],"all")))
view(2)
title("Magnitude")
subplot(2,2,4)
imagesc(angle(echoScene)')
view(2)
title("Phase")

%% Range Compression

Beta_coeff = 2.5;
tau_baseband = tau_centered' - 2*R_eta_c/c;
baseband_chirp = exp(1j.*pi.*K_r.*(tau_baseband).^2);
baseband_chirp = baseband_chirp((abs(tau_baseband) <= T_r/2));

nChirp = length(baseband_chirp);
win_kaiser = ifftshift(kaiser(N_rg,Beta_coeff));

MF = conj(fft(baseband_chirp,N_rg)).*win_kaiser;

s_rc = ifft(fft(echoScene,[],1).*MF,[],1);
s_rc = s_rc(1:end-nChirp+1,:);
% s_rc = s_rc(40:40+100-1,:); %to match the book

%% Compression Plots

figure
subplot(1,2,1)
imagesc(real(s_rc)')
view(2)
title("Real part")
subplot(1,2,2)
imagesc(20.*log10(abs(s_rc)'./max(abs(s_rc),[],"all")))
view(2)
title("Magnitude")

%% Azimuth FFT
S1_pre_src = fft(s_rc,[],2);

f_base = [0:N_az/2-1, -N_az/2:-1]*F_a/N_az; % fft ordering
f_eta = f_eta_c + mod(f_base - f_eta_c + F_a/2, F_a) - F_a/2; % unwrapped around centroid

tau_zero_center = tau_centered(1:Nr) - 2*R_eta_c/c;
f_tau = ifftshift(tau_zero_center*(F_r^2)/Nr)';

R0_eta_c = R_eta_c*cos(theta_r_c);
D = sqrt(1 - (lambda*f_eta./(2*V_r)).^2);
K_src = (2*(V_r^2)*(f0.*D).^3)./(c*R0_eta_c*f_eta.^2); 

K_m = K_r./(1-(K_r./K_src));

src_filter = exp(-1j.*pi.*(f_tau.^2)./K_src);
S1 = ifft(fft(S1_pre_src,[],1).*src_filter,[],1);

figure
subplot(2,2,1)
imagesc(real(S1_pre_src)')
view(2)
title("Real part")
subplot(2,2,2)
imagesc(20.*log10(abs(S1_pre_src)'./max(abs(S1_pre_src),[],"all")))
view(2)
title("Magnitude")
subplot(2,2,3)
imagesc(real(S1)')
view(2)
title("Real part")
subplot(2,2,4)
imagesc(20.*log10(abs(S1)'./max(abs(S1),[],"all")))
view(2)
title("Magnitude")

%% RCMC Sinc Interp

Nr = size(S1,1);
R0 = (range_centered(1:Nr).'*cos(theta_r_c));          % Nr x 1
delta_R = R0.*((1-D)./D);          % Nr x N_az
% delta_R = delta_R_centered - lambda^2 * R0 .* f_eta_c^2 / (8*V_r^2);


delta_R_idx = delta_R / scene_delta_R;
S2 = zeros(size(S1));

for jj = 1:N_az
    interp_len = 4;
    n_sam = 1:Nr;
    S1_sam = S1(:,jj);
    n_interp = 1:1/interp_len:Nr;
    [N_INTERP,N_SAM] = ndgrid(n_interp,n_sam);

    S1_interp = sinc(N_INTERP - N_SAM)*S1_sam;

    i = mod(round((n_sam' + delta_R_idx(:,jj) - 1)*interp_len) - 1, length(n_interp)) + 1;
    bad = i < 1 | i > length(n_interp);
    i(bad) = 1;

    S2(:,jj) = S1_interp(i);
    S2(bad,jj) = 0;
end

figure
subplot(1,2,1)
imagesc(real(S2)')
view(2)
title("Real part")
subplot(1,2,2)
imagesc(20.*log10(abs(S2)'./max(abs(S2),[],"all")))
view(2)
title("Magnitude")

%% RCMC Phase multiplier
f_tau = ifftshift(-floor(Nr/2):ceil(Nr/2)-1).' * F_r/Nr;
delta_R_invariant = R_eta_c * cos(theta_r_c) * ((1-D)./D);
G_rcmc = exp(1j.*4.*pi.*f_tau.*delta_R_invariant./c);

S2_PM = ifft(fft(S1,[],1).*G_rcmc,[],1);
figure
subplot(1,2,1)
imagesc(real(S2_PM)')
view(2)
title("Real part")
subplot(1,2,2)
imagesc(20.*log10(abs(S2_PM)'./max(abs(S2_PM),[],"all")))
view(2)
title("Magnitude")

%% Azimuth Compresison
H_az = exp(4j.*pi.*R0.*D.*f0/c);
Beta_coeff = 2.5;
x = 2*(f_eta - f_eta_c)/F_a;                                   % -1..1 across the unwrapped band
win_az = besseli(0, Beta_coeff*sqrt(max(1-x.^2,0))) / besseli(0,Beta_coeff);   % 1 x N_az
S3 = S2 .* (H_az .* win_az);

T_a = delta_f_dop*R_eta_c*lambda/(2*V_r^2);
nAzimuth = round(T_a*F_a);
nThrowaway = (nAzimuth-1)/2;

s_ac = ifft(S3,[],2);
s_ac = s_ac(:,1+nThrowaway:end-nThrowaway);

peaks_tf = islocalmax2(abs(s_ac),MaxNumExtrema=3);
[range_coord, azimuth_coord] = meshgrid(1:size(s_ac,1),1:size(s_ac,2));

range_coord = range_coord';
azimuth_coord = azimuth_coord';


%%
figure
imagesc(20.*log10(abs(s_ac)'./max(abs(s_ac),[],"all")))
hold on
plot3(range_coord(peaks_tf),azimuth_coord(peaks_tf),abs(s_ac(peaks_tf)),"red.",MarkerSize=12)
view(2)
title("Magnitude")
