% % %% installs required python packages in system-wide python3 installation
% !python3 -m pip install --upgrade pip
% !python3 -m pip install twopilabs.sense.x1000
%run in powershell
%% imports twopilabs python package 
pyenv("Version",".venv\Scripts\python.exe") % select python interpreter to be used in matlab (global here)
pyModule = py.importlib.import_module('twopilabs.sense.x1000'); %select twopilabs package
py.importlib.reload(pyModule); %import twopilabs package
%%
function mat = np2mat2(np)
    mat = double(py.numpy.asarray(np));
end


%% establish sensor connection
devices = pyModule.SenseX1000.find_devices(); % call find_devices routine to detect devices on USB and ETH
%%
devices = devices(1); % needed for matlab / python handling
device = pyModule.SenseX1000.open_device(devices{1}); % select first detected device
device.calc.trace_list(py.list({0,1,2}));

%% set device parameters 
% (including casts to python variable types, necessity depends on matlab /
% python version)
device.sense.frequency_start(py.float(126e9)); % ramp start frequency
device.sense.frequency_stop(py.float(182e9)); % ramp stop frequency
device.sense.sweep_time(py.float(1e-3)); % sweep duration
device.sense.sweep_count(py.int(16)); % number of sweeps to be done on single trigger
for i=1:100
%% do measurement^1111111111111111111111111111111111111111111111111111111111111111
acq = device.initiate.immediate_and_receive(); % arm & immediately trigger device
acqData = acq.read(); % read receive data structure

%% evaluate data
%(np2mat2() needed to cast numpy arrays to matlab variables, 
% may also depend on matlab version)

axis_frequency = np2mat2(acqData.freq_axes{1}); % instantaneous transmit frequency axis
axis_adctime = np2mat2(acqData.time_axes{1}); % adc time axis
data_ramp1 = np2mat2(acqData.array); %ramp data [sweep_count x channels x adc_samples]
data_ramp = squeeze(data_ramp1(:,1,:)); % select main txrx channel

F = numel(axis_adctime); % number of adc_samples

figure(1)
plot(axis_adctime,data_ramp) % plot IF signals

NFFT = 8192; % zero-padding
window = hanning(F); % select hanning-window
data_echo = ifft(mean(data_ramp,1).' .* window,NFFT,1); % do ifft
axis_range = 0.5 * ((1:NFFT)-1)/NFFT * 3e8 /(axis_frequency(2)-axis_frequency(1)); % single-trip distance axis

figure(2)
plot(axis_range,20*log10(abs(data_echo))); % plot echos
xlim([0 axis_range(end)/2])
% xlim([0 1])
% axis tight

% filename=sprintf('SAR2_test%d',i);
% save(filename,'data_ramp1');

pause(1);

end