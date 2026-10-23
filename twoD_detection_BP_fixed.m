% clear all
% close all
% clc
%% C-SAR Tomography
%% Parameter definition
% Find a way to properly define the robotic arm moving
% Define the RF and Digital Processing parameters to have unambigous
%   acquisition
% Other stuff
% fc = 140GHz
% BW = 50 GHz
% We will have IF samples, so we need to find a way to define those

%% Scene definition

%% Target generation
% Add some shapes

%% Signal generation (IF samples) (to be reviewed)

%% Range Compression (since we are using IF samples I think a Range FFT suffices?)

%% Back Projection (basically interpolate the RX signal from K-Space to actual geometry)
% Instead of projecting the range into each pixel, lets interpolate what
% each pixel should look like.

%% Possible new additions to increase resolution?

%% TO TRY INSTEAD
% SpotLight SAR (focused like C-SAR but moves linearly)
% Inverse SpotLight SAR
% Inverse StripMap SAR

%%
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
N_x         = 256;
N_y         = 256;
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

delta_x = bin_delta_R;
x_coords_idx = 1:N_x;
x_coords = (x_coords_idx - 1 - N_x/2)*delta_x;
y_coords_idx = 1:N_x;
y_coords = (y_coords_idx - 1 - N_y/2)*delta_x;

[X,Y] = ndgrid(x_coords,y_coords);

pixels = X+1j.*Y;

scene_rad = max(abs(x_coords));


rot_angles = 2*pi*(n_az-1)/N_az;

rad_drone = R_eta_c * cos(theta_r_c); %maybe in 3d changes
pos_drone = rad_drone .* exp(1j.*rot_angles);

% V_tan = pos_drone - circshift(pos_drone,1,2);  %creates problems for Ka
% V_tan = sum(abs(V_tan(1,:) + 1j*V_tan(2,:)))/N_az;

echoScene_BP = zeros(N_rg,N_az);

%% Chirp Definition (Rx)
nTargets = 3;
targets = zeros(1,nTargets);
for tt = 1:3
    x_tg_idx = randi([1, N_x]);
    y_tg_idx = randi([1, N_y]);
    pos_tg = pixels(x_tg_idx,y_tg_idx);
    while abs(pos_tg) >= scene_rad  % can be optimized by using R*exp(arg)
        x_tg_idx = randi([1, N_x]);
        y_tg_idx = randi([1, N_y]);
        pos_tg = pixels(x_tg_idx,y_tg_idx);
    end
    targets(tt) = pos_tg;

    R_tg = abs(pos_tg - pos_drone);
    
    [mag ,closest_idx] = min(R_tg);
    
    eta_tg = eta_centered(closest_idx);
    
    A0 = 1;
    
    for ii = 1:length(eta_centered) %captures

        % K_a = 2*V_r^2*cos(theta_r_c)^2./(lambda*R_tg(ii)); % Not quite correct, but will do for now
        eta = eta_centered(ii);
    
        R_eta = R_tg(ii)*cos(theta_r_c); % doubt the cos is needed here
        
        w_r_shifted = abs(tau_centered - dist2time(R_eta)) <= T_r/2;
        % w_a = sinc(0.886.*K_a.*(eta - eta_tg)/delta_f_dop).^2; %antenna
        w_a = 1;
    
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

s_rc = ifft(fft(echoScene_BP,[],1).*MF,[],1);
s_rc = circshift(s_rc,round(nChirp/2));
Nr = size(s_rc,1);

%% Plots
s_rc_dB = 20.*log10(abs(s_rc)./max(abs(s_rc),[],"all"));

figure
subplot(1,2,1)
imagesc(real(s_rc)')
view(2)
title("Real part")
subplot(1,2,2)
imagesc(s_rc_dB')
view(2)
title("Magnitude")



%% BP
theta_max = asin(scene_rad / rad_drone);

theta_angles = linspace(theta_max, -theta_max, 1000);

grid_mag = zeros(N_x,N_y);

oversampling = 4;
n_sam = (n_rg)-1;
n_interp = (1-(2/oversampling)):1/oversampling:(Nr + (2/oversampling));

ranges = (min(range_centered_0)-(2*bin_delta_R/oversampling)):bin_delta_R/oversampling:(max(range_centered_0)+(2*bin_delta_R/oversampling));
[N_INTERP,N_SAM] = ndgrid(n_interp,n_sam);

for nn = 1:N_az
    range_compressed_samples = abs(s_rc(:,nn));
    range_interped = sinc(N_INTERP - N_SAM)*range_compressed_samples;

    for rr = 1:length(ranges)
        possible_pos = pos_drone(nn) - ranges(rr).*exp(1j.*(theta_angles + rot_angles(nn)));
        possible_bins = possible_pos./delta_x;
    
        x_bin = round(real(possible_bins)) + 1 + N_x/2;
    
        x_bin_shift_good = ((x_bin>=1) & (x_bin<=N_x));
    
        y_bin = round(imag(possible_bins)) + 1 + N_y/2;
    
        y_bin_shift_good = (y_bin>=1) & (y_bin<=N_y);
    
        bin_shifts = y_bin_shift_good & x_bin_shift_good;
    
    
        y_bin = y_bin(bin_shifts);
        x_bin = x_bin(bin_shifts);

        % legit_angles = rot_angles(bin_shifts);
        % w_a = sinc(legit_angles);  %Some formula to relate the aperture
        % to the assignments is needed
        w_a = 1/length(x_bin);
    
        for xx = 1:length(x_bin)
            grid_mag(x_bin(xx),y_bin(xx)) = grid_mag(x_bin(xx),y_bin(xx)) + w_a*range_interped(rr)/length(pos_drone);
        end
        1;
    end
end
%%
ramp_filter = triang(N_x);

ramp_threshold = 1;
ramp_peak = (1 + ramp_threshold)/2;

mask = ramp_filter > ramp_threshold;
uu = (ramp_filter(mask) - ramp_threshold)/(1-ramp_threshold);

ramp_filter(mask) = ramp_threshold + (ramp_peak-ramp_threshold)*(2*uu - uu.^2);

image_sharp = ifft2(fft2(grid_mag).*ramp_filter.*ramp_filter');

%% Final Plot

figure
subplot(1,2,1)
imagesc(abs(grid_mag)'/max(abs(grid_mag),[],"all"))
view(2)
subplot(1,2,2)
imagesc(abs(image_sharp)'/max(abs(image_sharp),[],"all"))
view(2)
