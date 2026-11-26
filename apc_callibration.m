%% Internal Range Delay (R_int) & APC Calibration
% Separates the angle-independent internal RF delay (internal_offset, R_int)
% from the geometric recess between the black housing front face and the bare
% metal open-waveguide APC (housing_recess_offset, d_y), checks the range
% scale slope, and saves SI values [m] to range_offset.mat for
% Stripmap_SAR_TDBP.m.

clear all
close all
clc

%% Measured boresight data [cm]
% x_cm: tape-measure distance from black housing front face to reflector tape mark [cm]
x_cm = [
    22.5
    25
    27.5
    30
    32.5
    35
    37.5
    40
    42.5
    45
    47.5
    50];

% y_cm: radar-reported one-way range at each position [cm]
% (For best accuracy, extract peaks with a zero-padded FFT + parabolic fit
% instead of unpadded 2.68 mm FFT bins.)
y_cm = [
    2703
    2971
    3212
    3452
    3720
    3961
    4202
    4469
    4710
    4951
    5219
    5460] / 100;

reflector_offset_cm = 4.3;                       % Reflector front-to-apex depth [cm]
reflector_front_face2tape_measure_point_cm = 3.092; % Distance from reflector front face back to tape mark [cm]
housing_recess_offset_cm = 0.098;                % Bare metal waveguide recess behind black face [cm]
                                                 % (+ if recessed behind x = 0; - if protruding)

%% Convert to SI units [m]
assert(numel(x_cm) == numel(y_cm) && numel(x_cm) >= 3, ...
    "x_cm and y_cm must have the same length (>= 3 points).");

N = numel(x_cm);
R_meas = y_cm(:) / 100;
reflector_offset = reflector_offset_cm / 100;
reflector_front_face2tape_measure_point = reflector_front_face2tape_measure_point_cm/100;
housing_recess_offset = housing_recess_offset_cm / 100;

R_true_target = x_cm(:) / 100 + reflector_offset - reflector_front_face2tape_measure_point;
R_true_apc = R_true_target + housing_recess_offset;

%% Unit-slope internal delay estimate & unbiased statistics
offsets_housing = R_meas - R_true_target;
offsets_apc = R_meas - R_true_apc;

apc_offset_from_housing = mean(offsets_housing); % R_int + d_y [m]
internal_offset = mean(offsets_apc);             % Pure R_int [m] (angle-independent)

residuals_m = offsets_apc - internal_offset;
internal_offset_variance = var(offsets_apc, 0);  % Unbiased sample variance [m^2]
internal_offset_std = std(offsets_apc, 0);       % Per-measurement std [m]
internal_offset_se = internal_offset_std / sqrt(N); % Standard error of the mean [m]

internal_offset_sigma_mm = internal_offset_std * 1e3;
internal_offset_se_mm = internal_offset_se * 1e3;

%% Linear regression check: R_meas = range_scale * R_true_apc + linear_fit_offset
A = [R_true_apc, ones(N, 1)];
p = A \ R_meas;
range_scale = p(1);
linear_fit_offset = p(2);

fit_residuals = R_meas - A * p;
s2 = sum(fit_residuals.^2) / (N - 2);
cov_p = s2 * ((A' * A) \ eye(2));
range_scale_se = sqrt(cov_p(1, 1));
linear_fit_offset_se = sqrt(cov_p(2, 2));

if abs(range_scale - 1) > max(0.01, 3 * range_scale_se)
    warning("Fitted range scale (%.5f +/- %.5f) deviates from 1.000; check frequency/range axis.", ...
        range_scale, range_scale_se);
end

%% Save calibration to range_offset.mat (compatible with Stripmap_SAR_TDBP.m)
script_dir = fileparts(mfilename("fullpath"));
if isempty(script_dir)
    script_dir = pwd;
end
output_file = fullfile(script_dir, "range_offset.mat");

save(output_file, ...
    "internal_offset", ...
    "housing_recess_offset", ...
    "apc_offset_from_housing", ...
    "reflector_offset", ...
    "reflector_front_face2tape_measure_point", ...
    "internal_offset_variance", ...
    "internal_offset_std", ...
    "internal_offset_se", ...
    "internal_offset_sigma_mm", ...
    "internal_offset_se_mm", ...
    "range_scale", ...
    "range_scale_se", ...
    "linear_fit_offset", ...
    "linear_fit_offset_se", ...
    "R_true_apc", ...
    "R_meas", ...
    "residuals_m");

%% Print summary
fprintf("=== Radar Range & APC Calibration (N = %d) ===\n", N);
fprintf("Total offset from black housing (R_int + d_y): %.4f cm (%.6f m)\n", ...
    apc_offset_from_housing * 100, apc_offset_from_housing);
fprintf("Housing-to-waveguide recess (d_y):             %.2f mm (%.4f m)\n", ...
    housing_recess_offset * 1e3, housing_recess_offset);
fprintf("Internal RF delay (internal_offset, R_int):    %.4f cm (%.6f m)\n", ...
    internal_offset * 100, internal_offset);
fprintf("Per-point std (unbiased):                      %.3f mm\n", internal_offset_sigma_mm);
fprintf("Std error of mean (SE):                        +/- %.3f mm\n", internal_offset_se_mm);
fprintf("Linear fit scale check (ideal = 1.000):        %.6f +/- %.6f\n", ...
    range_scale, range_scale_se);
fprintf("Saved calibration to: %s\n", output_file);

%% Diagnostic plot
figure("Name", "APC & Internal Range Delay Calibration");
subplot(2, 1, 1);
plot(R_true_apc * 100, R_meas * 100, "o", "LineWidth", 1.2, "DisplayName", "Measured");
hold on;
plot(R_true_apc * 100, (R_true_apc + internal_offset) * 100, "-", "LineWidth", 1.2, ...
    "DisplayName", sprintf("R_{true,APC} + %.3f cm", internal_offset * 100));
grid on;
xlabel("True distance from bare metal waveguide APC [cm]");
ylabel("Radar-measured range [cm]");
title("Boresight Range Calibration");
legend("Location", "northwest");

subplot(2, 1, 2);
stem(R_true_apc * 100, residuals_m * 1e3, "filled", "LineWidth", 1.2);
yline(0, "--k");
grid on;
xlabel("True distance from bare metal waveguide APC [cm]");
ylabel("Residual [mm]");
title(sprintf("Residuals (\\sigma = %.2f mm, SE = \\pm%.2f mm)", ...
    internal_offset_sigma_mm, internal_offset_se_mm));