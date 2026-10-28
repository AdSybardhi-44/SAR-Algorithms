clear all
close all
clc


%% SAR Tomography
%% Parameter definition
% Find a way to properly define the robotic arm moving
% Define the RF and Digital Processing parameters to have unambigous
%   acquisition
% Other stuff
% fc = 140GHz
% BW = 50 GHz
% We will have IF samples, so we need to find a way to define those, IF
% samples need to be used only if we expect that the B_IF < B_RF
% K_r*T_r > K_r*delta_tau_d , meaning the pulse is longer than the delay
% difference from the swath. For the simulation only, IRL increase the
% sampling rate or change to RF methods.

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