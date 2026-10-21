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
f_nc        = 320;      % Hz , doppler centroid freq
% theta_r_c = 21.9; eta_c = -49.7; f_nc = 1975;
c      = 3e8;
lambda = c/f0;
theta  = deg2rad(theta_r_c);

%% Target definition
% Each target is described by:
%   R0_t        : slant range of closest approach
%   eta_cross_t : azimuth time at which the target is crossed by the beam center
% The time of closest approach then follows from the squint geometry:
%   eta0_t = eta_cross_t + R0_t*tan(theta)/V_r
% so that the beam-center crossing time is exactly eta_cross_t for every target.

% Target B (reference, scene center): R(eta_c) = R_eta_c, beam center crossed at eta_c
R0_B        = R_eta_c*cos(theta);
eta_cross_B = eta_c;

% Target A: same closest-approach range as B, different azimuth position.
% It is displaced in azimuth by d_eta (A is crossed by the beam center
% d_eta seconds later than B, i.e. V_r*d_eta metres along-track).
d_eta       = 0.3;                    % s
R0_A        = R0_B;                   % same R0 as B
eta_cross_A = eta_cross_B + d_eta;    % different azimuth position

% Target C: crosses the beam center at the same azimuth time as B,
% but at a different range (d_R further away).
% Because of the squint, equal beam-center crossing time at a different R0
% means a different closest-approach time (handled via eta0_t below),
% i.e. C lies on the same squinted line-of-sight as B, not at the same
% along-track position as B.
d_R         = 100;                    % m
R0_C        = R0_B + d_R;             % different range
eta_cross_C = eta_cross_B;            % same beam-center crossing time as B

R0_t        = [R0_A, R0_B, R0_C];
eta_cross_t = [eta_cross_A, eta_cross_B, eta_cross_C];
eta0_t      = eta_cross_t + R0_t*tan(theta)/V_r;   % closest-approach times
names       = {'A','B','C'};

%% Time axes
eta = eta_c + (-N_az/2 : N_az/2-1).'/F_a;         % azimuth time (N_az x 1)
tau = 2*R_eta_c/c + (-N_rg/2 : N_rg/2-1)/F_r;     % range time (1 x N_rg)
[TAU, ETA] = meshgrid(tau, eta);

%% Demodulated received signal (sum over targets)
s0 = zeros(N_az, N_rg);
for k = 1:numel(R0_t)
    R0 = R0_t(k);

    % Range history of target k (hyperbolic, closest approach at eta0_t(k))
    R_eta = sqrt(R0^2 + (V_r*(ETA - eta0_t(k))).^2);

    % Azimuth FM rate and exposure time of target k
    Rc  = R0/cos(theta);
    K_a = 2*V_r^2*cos(theta)^2/(lambda*Rc);
    T_a = delta_f_dop/K_a;

    % Envelopes
    t_d = TAU - 2*R_eta/c;                        % fast time relative to echo delay
    w_r = abs(t_d) <= T_r/2;                      % range window
    x   = 0.886*(ETA - eta_cross_t(k))/T_a;       % antenna pattern centred on beam crossing of target k
    w_a = (sin(pi*x+eps)./(pi*x+eps)).^2;

    s0 = s0 + w_r .* w_a ...
            .* exp(-1j*4*pi*f0*R_eta/c) ...       % azimuth phase
            .* exp( 1j*pi*K_r*t_d.^2);            % range chirp

    fprintf('Target %s: R0 = %.1f m, eta0 = %.3f s, beam-center crossing = %.3f s\n', ...
            names{k}, R0, eta0_t(k), eta_cross_t(k));
end

%% Expected target positions in the (tau, eta) plane (at beam-center crossing)
tau_cross_t = 2*(R0_t/cos(theta))/c;              % echo delay at beam-center crossing

%% Plots
figure;

subplot(2,2,1)
imagesc(tau*1e6, eta, real(s0)); axis xy
xlabel('\tau (\mus)'); ylabel('\eta (s)'); title('Real part'); colorbar

subplot(2,2,2)
imagesc(tau*1e6, eta, abs(s0)); axis xy; hold on
plot(tau_cross_t*1e6, eta_cross_t, 'wo', 'MarkerSize', 8, 'LineWidth', 1.5);
for k = 1:numel(names)
    text(tau_cross_t(k)*1e6 + 0.05, eta_cross_t(k), names{k}, 'Color', 'w', 'FontWeight', 'bold');
end
xlabel('\tau (\mus)'); ylabel('\eta (s)'); title('Magnitude (o = beam-center crossing)'); colorbar

subplot(2,2,3)
[~,iaz] = min(abs(eta - eta_c));
plot(tau*1e6, real(s0(iaz,:)), tau*1e6, imag(s0(iaz,:))); grid on
xlabel('\tau (\mus)'); title('Range line at \eta_c (B and C chirps)'); legend('Re','Im')

subplot(2,2,4)
[~,irg] = min(abs(tau - tau_cross_t(2)));         % range bin of target B
plot(eta, angle(s0(:,irg))); grid on
xlabel('\eta (s)'); title('Azimuth phase (wrapped) at range bin of B')