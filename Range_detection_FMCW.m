clear all; close all; clc;
%%

fc = 80e9;
gamma = 20e12;
Tp  = 2.5e-6;
B = Tp*gamma;
alpha_sampling = 1.07;
fs = B*alpha_sampling;
T = 1;
c  = 3e8;

scene_samples = 2048;

% Two separate time vectors
t_scene = linspace(0,10*Tp,1e4);              % scene duration
chirp_times = mod(t_scene,Tp) - Tp/2;

% K = B/T;

r_tg = 50; %m
%%

s_tx = [cos(2*pi*fc*t_scene + pi.*gamma.*(chirp_times).^2);sin(2*pi*fc*t_scene + pi.*gamma.*(chirp_times).^2)];

figure
hold on
plot(t_scene,s_tx(1,:))

%%
chirp_times_shift = mod(t_scene-(2*r_tg/c),Tp) - Tp/2;
s_rx = [cos(2*pi*fc*t_scene + pi.*gamma.*(chirp_times_shift).^2);sin(2*pi*fc*t_scene + pi.*gamma.*(chirp_times_shift).^2)].*(t_scene>=(2*r_tg/c));
plot(t_scene, s_rx(1,:).*s_tx(1,:))
%%

s_rx = exp(2j.*pi.*gamma.*t)

plot(angle(s_tx) + angle(s_rx))
