%%
clear all
close all
clc
%% Simulating Stripmap SAR
%% Parameters
res_rg = 2.7;
res_az = 1.7;
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
% theta_r_c = 21.9; eta_c = -49.7; f_eta_c = 1975;
c      = 3e8;
lambda = c/f0;
theta_r_c  = deg2rad(theta_r_c);
La = 2*res_az;
theta_bw = 0.886*lambda/La;

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

% K-space Parameters
delta_R = dist2time(1/F_r,"dist");
range_res = c/(2*abs(K_r)*T_r);
n_rg = 1:N_rg;
n_rg_centered = (n_rg-1-N_rg/2);
range_baseband = n_rg_centered*delta_R; % swath relative to the center
range_centered = range_baseband + R_eta_c; % swath relative to the origin
range_baseband_0 = range_baseband*cos(theta_r_c);
range_centered_0 = range_centered*cos(theta_r_c);
tau_baseband = dist2time(range_baseband);
tau_centered = dist2time(range_centered);

n_az = 1:N_az;
n_az_centered = (n_az-1-N_az/2);
eta_baseband = (n_az-1-N_az/2)/F_a;
eta_centered = eta_baseband + eta_c;

% Cartesian Parameters
pos_drone = eta_centered*V_r + 1j.*0;

Ls = real(pos_drone(end) - pos_drone(1));

delta_x = V_r/F_a;
N_x = N_az;
x_coords_idx = 1:N_x;
x_sc = V_r*eta_c + R_eta_c*sin(theta_r_c);
x_coords = eta_baseband*V_r + x_sc;

delta_y = delta_R;
N_y = N_rg;
y_coords_idx = 1:N_y;
y_sc = R_eta_c*cos(theta_r_c);
y_coords = range_centered_0; %approximately since distance from center much larger than the swath

[X,Y] = ndgrid(x_coords,y_coords);

pixels = X+1j.*Y;

%% Load arbitrary grayscale reflectivity map 
img = imread('durres_scene.png');

% Convert RGB -> grayscale if necessary
if ndims(img) == 3
    img = rgb2gray(img);
end

img = double(img);

% Normalize reflectivities to [0,1]
img = img ./ max(img(:));

% Optional threshold to remove weak pixels
threshold = 0.05;
img(img < threshold) = 0;

% Resize to SAR grid dimensions
target_map = imresize(img',[N_x N_y],'bilinear');

[x_tg_idx, y_tg_idx] = find(target_map ~= 0);

target_amplitude = target_map( ...
    sub2ind(size(target_map), x_tg_idx, y_tg_idx));

nTargets = numel(x_tg_idx);

fprintf('Number of point targets: %d\n', nTargets);


%% Echo generation from reflectivity map
echoScene_BP_no_noise = complex(zeros(N_rg,N_az));

targets = complex(zeros(2,nTargets));

for tt = 1:nTargets

    ix = x_tg_idx(tt);
    iy = y_tg_idx(tt);

    pos_tg = pixels(ix,iy);
    A0 = target_amplitude(tt);

    % Store physical position and pixel index
    targets(1,tt) = pos_tg;
    targets(2,tt) = ix + 1j*iy;

    % Target-platform geometry over slow time
    rel_tg = pos_tg - pos_drone;

    R_tg = abs(rel_tg);

    dx = real(rel_tg);
    dy = imag(rel_tg);

    theta_tg = atan2(dx,dy);

    % More accurate direction-cosine antenna argument
    delta_u = sin(theta_tg) - sin(theta_r_c);

    w_a = sinc((La/lambda)*delta_u).^2;

    tau_delay = dist2time(R_tg);

    dtau = tau_centered' - tau_delay;

    w_r_shifted = ...
        abs(dtau) <= (T_r/2);

    echo = ...
        (A0.*w_a.*exp(-1j*4*pi*f0*R_tg/c)) .* ...
        (w_r_shifted.*exp(1j*pi*K_r* dtau.^2));

    echoScene_BP_no_noise = ...
        echoScene_BP_no_noise + echo;
end

% %% SAR Letters
%% Target reflectivity map
% target_map = zeros(N_x, N_y);
% 
% % 7-by-5 binary font
% letter_S = [
%     1 1 1 1 1
%     1 0 0 0 0
%     1 0 0 0 0
%     1 1 1 1 1
%     0 0 0 0 1
%     0 0 0 0 1
%     1 1 1 1 1
% ];
% 
% letter_A = [
%     0 1 1 1 0
%     1 0 0 0 1
%     1 0 0 0 1
%     1 1 1 1 1
%     1 0 0 0 1
%     1 0 0 0 1
%     1 0 0 0 1
% ];
% 
% letter_R = [
%     1 1 1 1 0
%     1 0 0 0 1
%     1 0 0 0 1
%     1 1 1 1 0
%     1 0 1 0 0
%     1 0 0 1 0
%     1 0 0 0 1
% ];
% 
% % Enlarge each binary pixel into a block
% scale_x = 4;
% scale_y = 4;
% 
% S_big = kron(letter_S, ones(scale_x,scale_y));
% A_big = kron(letter_A, ones(scale_x,scale_y));
% R_big = kron(letter_R, ones(scale_x,scale_y));
% 
% % Gap between letters
% gap = zeros(size(S_big,1), 3*scale_y);
% 
% SAR_pattern = [S_big, gap, A_big, gap, R_big];
% 
% % Rotate if necessary so the text follows your preferred axes
% % SAR_pattern = rot90(SAR_pattern);
% 
% % Embed pattern near the center of the target map
% pattern_Nx = size(SAR_pattern,1);
% pattern_Ny = size(SAR_pattern,2);
% 
% x_start = round((N_x - pattern_Nx)/2) + 1;
% y_start = round((N_y - pattern_Ny)/2) + 1;
% 
% x_range = x_start:(x_start + pattern_Nx - 1);
% y_range = y_start:(y_start + pattern_Ny - 1);
% 
% target_map(x_range,y_range) = flipud(SAR_pattern);
% 
% [x_tg_idx, y_tg_idx] = find(target_map ~= 0);
% 
% target_amplitude = target_map( ...
%     sub2ind(size(target_map), x_tg_idx, y_tg_idx));
% 
% nTargets = numel(x_tg_idx);
% 
% fprintf('Number of point targets: %d\n', nTargets);
% 
% %% Echo generation from reflectivity map
% echoScene_BP_no_noise = complex(zeros(N_rg,N_az));
% 
% targets = complex(zeros(2,nTargets));
% 
% for tt = 1:nTargets
% 
%     ix = x_tg_idx(tt);
%     iy = y_tg_idx(tt);
% 
%     pos_tg = pixels(ix,iy);
%     A0 = target_amplitude(tt);
% 
%     % Store physical position and pixel index
%     targets(1,tt) = pos_tg;
%     targets(2,tt) = ix + 1j*iy;
% 
%     % Target-platform geometry over slow time
%     rel_tg = pos_tg - pos_drone;
% 
%     R_tg = abs(rel_tg);
% 
%     dx = real(rel_tg);
%     dy = imag(rel_tg);
% 
%     theta_tg = atan2(dx,dy);
% 
%     % More accurate direction-cosine antenna argument
%     delta_u = sin(theta_tg) - sin(theta_r_c);
% 
%     w_a = sinc((La/lambda)*delta_u).^2;
% 
%     for ii = 1:N_az
% 
%         R_eta = R_tg(ii);
% 
%         tau_delay = dist2time(R_eta);
% 
%         w_r_shifted = ...
%             abs(tau_centered - tau_delay) <= T_r/2;
% 
%         echo = ...
%             A0 .* ...
%             w_r_shifted.' .* ...
%             w_a(ii) .* ...
%             exp(-1j*4*pi*f0*R_eta/c) .* ...
%             exp(1j*pi*K_r* ...
%             (tau_centered.' - tau_delay).^2);
% 
%         echoScene_BP_no_noise(:,ii) = ...
%             echoScene_BP_no_noise(:,ii) + echo;
%     end
% end

% %% Chirp Definition (Rx) (random)
% 
% echoScene_BP_no_noise = zeros(N_rg,N_az);
% nTargets = 50;
% targets = zeros(2,nTargets);
% for tt = 1:nTargets
%     x_tg_idx = randi([1, N_x]);
%     y_tg_idx = randi([1, N_y]);
%     pos_tg = pixels(x_tg_idx,y_tg_idx); %maybe some pixels don't appear but that's ok
% 
%     targets(1,tt) = pos_tg;
%     targets(2,tt) = x_tg_idx+ 1j*y_tg_idx;
% 
%     rel_tg = pos_tg - pos_drone;
% 
%     R_tg = abs(rel_tg);
%     dx = real(rel_tg);
%     dy = imag(rel_tg);
% 
%     theta_tg = atan2(dx,dy);
% 
%     delta_theta = sin(theta_tg) - sin(theta_r_c);
% 
%     w_a = sinc((La/lambda)*delta_theta).^2;
% 
%     A0 = 1;
%     for ii = 1:length(eta_centered) %captures
% 
%         R_eta = R_tg(ii);
% 
%         w_r_shifted = abs(tau_centered - dist2time(R_eta)) <= T_r/2;
% 
%         echo = A0 .* w_r_shifted' .* w_a(ii) ...
%             .* exp(-1j.*4.*pi.*f0.*R_eta./c) ...
%             .* exp(1j.*pi.*K_r.*(tau_centered' - dist2time(R_eta)).^2);
% 
%         echoScene_BP_no_noise(:,ii) = echoScene_BP_no_noise(:,ii) + echo;
%     end
% end
% 
% %% Random Targets Map
% target_map = zeros(size(pixels));
% for tt = 1:nTargets
%     target_map(real(targets(2,tt)), imag(targets(2,tt))) = 1;
% end

%% 
figure
imagesc(y_coords, x_coords, target_map)
axis xy equal tight
colormap(gray)
colorbar
ylabel('Along-track x [m]')
xlabel('Ground-range y [m]')
title('Real reflectivity map')

filename = sprintf("Results/Stripmap/real_scene.png");
saveas(gcf,filename)

%% Simulations      
% for SNR = [Inf, 20, 0, -10, -20]
for SNR = Inf
    sig_power = mean(abs(echoScene_BP_no_noise(:)).^2);
    sigma_n = sqrt(sig_power*10^(-SNR/10));
    add_noise = sigma_n*(randn(N_rg,N_az) + 1j.*randn(N_rg,N_az))/sqrt(2);
    echoScene_BP = echoScene_BP_no_noise + add_noise;
    %% Plots
    figure
    sgtitle(sprintf("Rx Echoes SNR=%ddB",SNR))
    subplot(1,2,1)
    imagesc(tau_centered, n_az, real(echoScene_BP)')
    view(2)
    xlabel("Delays [s]")
    ylabel("Samples")
    title("Real part")
    subplot(1,2,2)
    imagesc(tau_centered, n_az, 20.*log10(abs(echoScene_BP)'./max(abs(echoScene_BP),[],"all")))
    view(2)
    xlabel("Delays [s]")
    ylabel("Samples")
    title("Magnitude")
    filename = sprintf("Results/Stripmap/rx_echo_%ddB.png",SNR);
    saveas(gcf,filename)
    
    %% Range Compression
    Beta_coeff = 2.5;
    
    baseband_chirp = exp(1j.*pi.*K_r.*(tau_baseband').^2);
    baseband_chirp = baseband_chirp((abs(tau_baseband) <= T_r/2));
    
    nChirp = length(baseband_chirp);

    Nfft = 2^nextpow2(N_rg+nChirp-1);
    win_kaiser = kaiser(nChirp,Beta_coeff); % behaves as the ramp needed, but requires higher beta, but adding a ramp for better performance
    % win_kaiser = ones(nChirp,1);

    padded_chirp = zeros(Nfft,1);
    padded_chirp(1:nChirp) = baseband_chirp.*win_kaiser;
    padded_chirp = circshift(padded_chirp,-floor(nChirp/2));
    MF = conj(fft(padded_chirp,Nfft,1));
    
    s_rc = ifft(fft(echoScene_BP,Nfft,1).*MF,Nfft,1);
    s_rc = s_rc(1:N_rg,:);
    Nr = size(s_rc,1);
    
    %% Plots
    s_rc_dB = 20.*log10(abs(s_rc)./max(abs(s_rc),[],"all"));
    s_rc_dB(s_rc_dB<-50) = -50;

    figure
    sgtitle(sprintf("Range Compressed Rx Echoes SNR=%ddB",SNR))
    subplot(1,2,1)
    imagesc(tau_centered, n_az, real(s_rc)')
    view(2)
    xlabel("Delays [s]")
    ylabel("Samples")
    title("Real part")
    subplot(1,2,2)
    imagesc(tau_centered, n_az, s_rc_dB')
    view(2)
    xlabel("Delays [s]")
    ylabel("Samples")
    title("Magnitude")
    filename = sprintf("Results/Stripmap/compressed_echo_%ddB.png",SNR);
    saveas(gcf,filename)
    
    %% BP Algorithm 1 (Slow but useful in memory limited situations)
    % grid_mag = zeros(N_x, N_y);
    % for xx = x_coords_idx
    %     for yy = y_coords_idx
    %         tg_pixel = pixels(xx,yy);
    %         rel_pixel = tg_pixel - pos_drone;
    %         R_pixel = abs(rel_pixel);
    %         N_INTERP = (R_pixel - range_centered(1))/delta_R +1;
    %         N_SAM = n_rg;
    %         for nn = n_az
    %             INTERP_VALUES = N_INTERP(nn) - N_SAM;
    %             interp_neighbors = 16;
    %             mask_interp = abs(INTERP_VALUES)<=interp_neighbors/2;
    %             INTERP_KERNEL = zeros(size(INTERP_VALUES));
    %             INTERP_WINDOW = hamming(interp_neighbors);
    %             INTERP_KERNEL(mask_interp) = sinc(INTERP_VALUES(mask_interp)).*(0.54 +0.46*cos(2*pi*INTERP_VALUES(mask_interp)/interp_neighbors));
    %             echoScene2Pixel = INTERP_KERNEL*(s_rc(:,nn))*exp(1j.*4.*pi.*f0.*R_pixel(nn)./c);
    % 
    %             grid_mag(xx,yy) = grid_mag(xx,yy) + echoScene2Pixel;
    %         end
    %     end
    % end

    %% BP Algorithm 2 (Faster than 1, uses vectorization of matrices, requires more memory)
    grid_mag = complex(zeros(N_x, N_y));
    half_win = 16;
    pixels_vec = pixels(:);
    weightSum = zeros(size(pixels_vec));
    for nn = 1:N_az
        rel_pixel = pixels_vec - pos_drone(nn);
        R_pixel = abs(rel_pixel);
        N_INTERP = (R_pixel - range_centered(1))/delta_R +1;
        UU = N_INTERP - n_rg;
        mask = abs(UU) <= half_win;
        KK = zeros(size(UU));
        UU_MASKED = UU(mask);
        KK(mask) = sinc(UU_MASKED);
        % KK = sinc(UU); %full interpolation
        s_interp = KK * s_rc(:,nn);
        theta_pixel = atan2(real(rel_pixel),imag(rel_pixel));
        delta_u = sin(theta_pixel) - sin(theta_r_c);
        w_a = sinc((La/lambda)*delta_u).^2;
        % phi = f0*dist2time(R_pixel) - 0.5*K_r*dist2time(R_pixel).^2; % if IF was used
        phi = f0*dist2time(R_pixel);
        echo2Pixel = s_interp .* exp(1j*2*pi*phi).* w_a;
        grid_mag = grid_mag + reshape(echo2Pixel, N_x, N_y);
        weightSum = weightSum + w_a;
    end
    grid_mag = grid_mag./ reshape(max(weightSum,1e-12), N_x, N_y);

    %% BP Algorithm 3 (Fastest but uses a lot of memory and specialized functions)
    % L_up = 16;
    % s_rc_up = interpft(s_rc, L_up*N_rg,1);
    % r_axis_up = range_centered(1) + (0:(L_up*N_rg -1))*(delta_R/L_up);
    % grid_mag = complex(zeros(N_x, N_y));
    % weightSum = zeros(N_x, N_y);
    % for nn = 1:N_az
    %     rel_pixel = pixels - pos_drone(nn);
    %     R_pixel = abs(rel_pixel);
    %     theta_pixel = atan2(real(rel_pixel),imag(rel_pixel));
    %     delta_u = sin(theta_pixel) - sin(theta_r_c);
    %     w_a = sinc((La/lambda)*delta_u).^2;
    % 
    %     s_interp = interp1(r_axis_up, s_rc_up(:,nn),R_pixel, "linear",0);
    %     % phi = f0*dist2time(R_pixel) - 0.5*K_r*dist2time(R_pixel).^2; % if IF was used
    %     phi = f0*dist2time(R_pixel);
    %     grid_mag = grid_mag + s_interp.*exp(1j*2*pi*phi).*w_a;
    %     weightSum = weightSum + w_a;
    % end
    % grid_mag = grid_mag./ max(weightSum,1e-12);

    %% Scene Recreated
    figure
    imagesc(y_coords, x_coords, abs(grid_mag)/max(abs(grid_mag),[],"all"))
    axis xy equal tight
    colormap(gca, gray)
    clim([0 1])
    hold on
    scatter3(y_coords(imag(targets(2,:))),x_coords(real(targets(2,:))),ones(1,nTargets),"red",".","LineWidth",2)
    ylabel('Along-track x [m]')
    xlabel('Ground-range y [m]')
    title(sprintf("Recreated Scene SNR=%d",SNR))
    filename = sprintf("Results/Stripmap/recreated_scene_%ddB.png",SNR);
    saveas(gcf,filename)

    figure
    mesh(20*log10(abs(grid_mag)'/max(abs(grid_mag),[],"all")))
    title(sprintf("Magnitude of each pixel SNR=%d",SNR))

    filename = sprintf("Results/Stripmap/magnitude_recreated_scene_%ddB.png",SNR);
    saveas(gcf,filename)

    
end
%% Range-Doppler Algorithm
% %% Azimuth FFT
%     S1 = fft(s_rc,[],2);
%     %% RCMC Sinc Interp
%     f_base = [0:N_az/2-1, -N_az/2:-1]*F_a/N_az; % fft ordering
%     f_eta = f_eta_c + mod(f_base - f_eta_c + F_a/2, F_a) - F_a/2; % unwrapped around centroid
% 
%     Nr = size(S1,1);
%     R0 = (range_centered(1:Nr).'*cos(theta_r_c));          % Nr x 1
%     delta_R_rcmc = lambda^2 * R0 .* (f_eta.^2) / (8*V_r^2);          % Nr x N_az
% 
% 
%     delta_R_idx = delta_R_rcmc / delta_R;
%     S2 = zeros(size(S1));
% 
%     for jj = 1:N_az
%         interp_len = 4;
%         n_sam = 1:Nr;
%         S1_sam = S1(:,jj);
%         n_interp = 1:1/interp_len:Nr;
%         [N_INTERP,N_SAM] = ndgrid(n_interp,n_sam);
% 
%         S1_interp = sinc(N_INTERP - N_SAM)*S1_sam;
% 
%         i = round((n_sam' + delta_R_idx(:,jj) - 1)*interp_len) + 1;
%         bad = i < 1 | i > length(n_interp);
%         i(bad) = 1;
% 
%         S2(:,jj) = S1_interp(i);
%         S2(bad,jj) = 0;
%     end
%     %% Azimuth Compresison
%     K_a = 2*V_r^2*cos(theta_r_c)^2./(lambda*range_centered(1:size(S2,1))); % same R0 from last section
%     H_az = exp(-1j.*pi.*(f_eta.^2)./K_a');
%     Beta_coeff = 2.5;
%     x = 2*(f_eta - f_eta_c)/F_a;                                   % -1..1 across the unwrapped band
%     win_az = besseli(0, Beta_coeff*sqrt(max(1-x.^2,0))) / besseli(0,Beta_coeff);   % 1 x N_az
%     S3 = S2 .* (H_az .* win_az);
% 
%     T_a = delta_f_dop*R_eta_c*lambda/(2*V_r^2);
%     nAzimuth = round(T_a*F_a);
%     nThrowaway = (nAzimuth-1)/2;
% 
%     s_ac = ifft(S3,[],2);
% 
%     figure
%     imagesc(flipud(abs(s_ac)')./max(abs(s_ac),[],"all"))
%     view(2)
%     colormap(gca, gray)
%     clim([0 1])
%     title("Magnitude")
% 
% 
