clear all
close all
clc
%%
x_coords_idx = 1:256;
y_coords_idx = 1:256;

delta_x = 1; %mm

x_coords = (delta_x*(x_coords_idx - 1 - 256/2));
y_coords = x_coords;

[X,Y] = ndgrid(x_coords,y_coords);

pixels = X+1j.*Y;

scene_rad = max(abs(x_coords));

rad_drone = 1e3;

rot_angles = linspace(0,2*pi,256);

drone_pos = rad_drone.*exp(1j*rot_angles);

theta_max = asin(scene_rad / rad_drone);

theta_angles = linspace(theta_max, -theta_max, 256);

ranges = abs(y_coords + rad_drone);
range_compressed_samples_eta = repmat(10*rand(1,256),length(rot_angles),1);

%% BackProjection on CSAR
grid_mag = zeros(256,256);
nTargets = 3;
targets = zeros(1,nTargets);

for tt = 1:nTargets
    true_pos = [randi(256),randi(256)];
    target_pos = pixels(true_pos(1),true_pos(2));
    while abs(target_pos) >=scene_rad
        true_pos = [randi(256),randi(256)];
        target_pos = pixels(true_pos(1),true_pos(2));
    end

    target_range = abs(target_pos - drone_pos);
    tg_range_bin = round((target_range - min(ranges))/delta_x) + 1; %should be delta_r, but just for proof of concept
    for dd = 1:length(drone_pos)
        range_compressed_samples_eta(dd,tg_range_bin(dd)) = range_compressed_samples_eta(dd,tg_range_bin(dd)) + 100*max(target_range)/target_range(dd);
    end
    targets(tt) = target_pos;
end

for dd = 1:length(drone_pos)
    range_compressed_samples = range_compressed_samples_eta(dd,:);

    for ii = 1:length(ranges)
        possible_pos = drone_pos(dd) - ranges(ii).*exp(1j.*(theta_angles + rot_angles(dd)));
        possible_bins = possible_pos./delta_x;
    
        x_bin = round(real(possible_bins)) + 1 + 256/2;
    
        x_bin_shift_good = ((x_bin>=1) & (x_bin<=256));
    
        y_bin = round(imag(possible_bins)) + 1 + 256/2;
    
        y_bin_shift_good = (y_bin>=1) & (y_bin<=256);
    
        bin_shifts = y_bin_shift_good & x_bin_shift_good;
    
    
        y_bin = y_bin(bin_shifts);
        x_bin = x_bin(bin_shifts);

        legit_angles = rot_angles(bin_shifts);
        % w_a = sinc(legit_angles);  %Some formula to relate the aperture
        % to the assignments
        w_a = 1/length(x_bin);
    
        for jj = 1:length(x_bin)
            grid_mag(x_bin(jj),y_bin(jj)) = grid_mag(x_bin(jj),y_bin(jj)) + w_a*range_compressed_samples(ii)/length(drone_pos);
        end
    end
end
%%

grid_mag_dB = 20*log10(grid_mag/max(grid_mag,[],"all"));

figure
subplot(1,2,1)
mesh(grid_mag')
% view(2)
subplot(1,2,2)
mesh(grid_mag_dB')




% hold on
% for tt = 1:nTargets
%     scatter(real(targets(tt)),imag(targets(tt)),100)
% end



% %%
% oversampling = 4;
% n_sam = (1:256)-1;
% n_interp = linspace(1,256,256*oversampling)-1;
% [N_INTERP,N_SAM] = ndgrid(n_interp,n_sam);
% range_interped = sinc(N_INTERP - N_SAM)*range_compressed_samples';
% 
% x_interp = delta_x*(n_interp - 256/2);
% y_interp = x_interp;
% drone_range_interp = -y_drone + y_interp;
% 
% 
% 
% %%
% figure
% hold on
% plot(ranges,range_compressed_samples)
% plot(drone_range_interp + delta_x/(2*oversampling),range_interped)
% 
% %%
% 
% [X_grid, Y_grid] = ndgrid(x_interp,y_interp) - delta_x/2;
% 
% drone2sub_px = X_grid-x_drone + 1j*(Y_grid-y_drone);
% R_drone2sub_px = abs(drone2sub_px);
% theta_drone2sub_px = angle(drone2sub_px) - pi/2;
% old_idx = R_drone2sub_px==1000+R_drone2sub_px;
% for ii = 1:length(range_interped)
%     bleed_idx = (R_drone2sub_px<=drone_range_interp(ii));
%     fix_idx = (bleed_idx~=old_idx);
%     old_idx = bleed_idx;
% end