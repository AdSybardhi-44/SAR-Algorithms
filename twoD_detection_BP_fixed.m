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

echoScene_BP = zeros(N_rg,N_az);

%% Chirp Definition (Rx)

rot_angles = 5*n_az_centered/N_az;
pos_drone = [cos(pi*rot_angles/180); -sin(pi*rot_angles/180)] * R_eta_c * cos(theta_r_c);
V_tan = pos_drone - circshift(pos_drone,1,2);
V_tan = sum(abs(V_tan(1,:) + 1j*V_tan(2,:)))/N_az;


focus_rad = (2*bin_delta_R^2)/lambda;

for jj = 1:1
    focus_region = range_baseband_0(range_baseband_0<=focus_rad*0.9);
    x_tg_idx = randi([1, length(focus_region)]);
    y_tg_idx = randi([1, length(focus_region)]);
    
    x_tg = focus_region(x_tg_idx);
    y_tg = focus_region(y_tg_idx);
    % x_tg = 0;
    % y_tg = 0;
    
    pos_tg = [x_tg; y_tg];
    
    re = x_tg-pos_drone(1,:);
    im = y_tg-pos_drone(2,:);

    R_tg = abs(re+1j*im);
    
    [mag ,closest_idx] = min(R_tg);
    
    eta_tg = eta_centered(closest_idx);
    
    A0 = 1;
    
    for ii = 1:length(eta_centered) %captures

        K_a = 2*V_r^2*cos(theta_r_c)^2./(lambda*R_tg(ii));
        eta = eta_centered(ii);
    
        R_eta = R_tg(ii)*cos(theta_r_c);
        
        w_r_shifted = abs(tau_centered - dist2time(R_eta)) <= T_r/2;
        w_a = sinc(0.886.*K_a.*(eta - eta_tg)/delta_f_dop).^2; %antenna
    
        echo = A0 .* w_r_shifted' .* w_a ...
            .* exp(-1j.*4.*pi.*f0.*R_eta./c) ...
            .* exp(1j.*pi.*K_r.*(tau_centered' - dist2time(R_eta)).^2);
    
        echoScene_BP(:,ii) = echoScene_BP(:,ii) + echo;
    end
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

%% Range Compression
Beta_coeff = 2.5;

baseband_chirp = exp(1j.*pi.*K_r.*(tau_baseband').^2);
baseband_chirp = baseband_chirp((abs(tau_baseband) <= T_r/2));

nChirp = length(baseband_chirp);
win_kaiser = ifftshift(kaiser(N_rg,Beta_coeff)); % behaves as the ramp needed, but adding a ramp for insurance
MF = conj(fft(baseband_chirp,N_rg)).*win_kaiser;

s_rc_BP = ifft(fft(echoScene_BP,[],1).*MF,[],1);
% s_rc_BP = circshift(s_rc_BP,round(nChirp/2));
Nr = size(s_rc_BP,1);

%% Plots
figure
subplot(1,2,1)
imagesc(real(s_rc_BP)')
view(2)
title("Real part")
subplot(1,2,2)
imagesc(20.*log10(abs(s_rc_BP)'./max(abs(s_rc_BP),[],"all")))
view(2)
title("Magnitude")

% %% BP 
% S1 = fft(s_rc_BP,[],2);
% 
% figure
% subplot(1,2,1)
% imagesc(real(S1)')
% view(2)
% title("Real part")
% subplot(1,2,2)
% imagesc(20.*log10(abs(S1)'./max(abs(S1),[],"all")))
% view(2)
% title("Magnitude")
% 
% f_base = [0:N_az/2-1, -N_az/2:-1]*F_a/N_az; % fft ordering
% f_eta = 0 + mod(f_base - 0 + F_a/2, F_a) - F_a/2; % unwrapped around centroid
% 
% f_tau = ifftshift(-floor(Nr/2):ceil(Nr/2)-1).' * F_r/Nr;
% 
% delta_R = lambda^2 .* range_centered_0(1:Nr)' .* cos(theta_r_c) .* f_eta.^2 / (8*V_r^2);
% delta_R_idx = delta_R / bin_delta_R;
% S2 = zeros(size(S1));
% 
% for jj = 1:N_az
% 
%     interp_len = 4;
%     n_sam = 1:Nr;
%     S1_sam = S1(:,jj);
%     n_interp = 1:1/interp_len:Nr;
%     [N_INTERP,N_SAM] = ndgrid(n_interp,n_sam);
% 
%     S1_interp = sinc(N_INTERP - N_SAM)*S1_sam;
% 
%     i = round((n_sam' + delta_R_idx(:,jj) - 1)*interp_len) + 1;
%     bad = i < 1 | i > length(n_interp);
%     i(bad) = 1;
% 
%     S2(:,jj) = S1_interp(i);
%     S2(bad,jj) = 0;
% end
% %%
% figure
% subplot(1,2,1)
% imagesc(real(S2)')
% view(2)
% title("Real part")
% subplot(1,2,2)
% imagesc(20.*log10(abs(S2)'./max(abs(S2),[],"all")))
% view(2)
% title("Magnitude")
% 
% 
% s_rc_BP = ifft(S2,[],2);
%% Plots
figure
subplot(1,2,1)
imagesc(real(s_rc_BP)')
view(2)
title("Real part")
subplot(1,2,2)
imagesc(20.*log10(abs(s_rc_BP)'./max(abs(s_rc_BP),[],"all")))
view(2)
title("Magnitude")


%% BP

ramp_filter = triang(Nr);

ramp_threshold = 0.9;
ramp_peak = (1 + ramp_threshold)/2;

mask = ramp_filter > ramp_threshold;
uu = (ramp_filter(mask) - ramp_threshold)/(1-ramp_threshold);

ramp_filter(mask) = ramp_threshold + (ramp_peak-ramp_threshold)*(2*uu - uu.^2);

s_ed = zeros(Nr,N_az);
rot_angles = 5*(n_az-1)/N_az;
for ii = 1:N_az

    s_ed_tau = ifft(fft(s_rc_BP(:,ii)).*ramp_filter);
    s_ed_tau_rep = repmat(s_ed_tau,1,N_az)/N_az;
    s_ed = s_ed + imrotate(real(s_ed_tau_rep),rot_angles(ii),"bilinear","crop");
    s_ed = s_ed + 1j.*imrotate(imag(s_ed_tau_rep),rot_angles(ii),"bilinear","crop");
end
%%

figure
imagesc(20.*log10(abs(s_ed)'./max(abs(s_ed),[],"all")))
hold on
plot3(x_tg_idx,y_tg_idx,1,"red.",MarkerSize=12)
view(2)
title("Magnitude")

[X, Y] = meshgrid(range_baseband_0,range_baseband_0);