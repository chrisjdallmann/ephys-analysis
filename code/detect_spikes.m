% DETECT_SPIKES.m calls a GUI for interactive spike detection 
%
% Functions/toolboxes required:
%    spike_annotation_gui_V30.m

% Author: Chris J. Dallmann 
% Affiliation: University of Wuerzburg
% Last revision: 07-August-2025

% ------------- BEGIN CODE -------------

clear
clc

% Settings
file_name = '2024_07_05_0004';
file_path = 'C:\Users\Sirin\Documents\MATLAB\von Sirin\RRflight'; 
sampling_rate = 20000;

% Load file
load([file_path, file_name, '.mat']);

% Get Vm (always first entry in data)
vm = data(:,1);

% Smooth membrane potential
vm = smooth(vm, 10);

% Set max recording length before splitting
max_rec_length = 60; % seconds

if length(vm) > max_rec_length * sampling_rate
    recording_length = floor(size(vm, 1)/3);
    AllPeakTimes_final = [];

    % Loop over parts
    for part = 1:3
      
        vm_current = vm(recording_length*part - recording_length+1 : recording_length*part);

        % Open spike detection GUI
        % Returns AllPeakTimes
        spike_annotation_gui_V30(vm_current)
        uiwait;

        % Determine current length of vm 
        if part == 1
            vm_length_1 = size(vm_current, 1)/sampling_rate;
        elseif part == 2
            vm_length_2 = size(vm_current, 1)/sampling_rate;
        end

        if part == 1
            AllPeakTimes_final = [AllPeakTimes_final; AllPeakTimes];
        elseif part == 2
            AllPeakTimes_final = [AllPeakTimes_final; AllPeakTimes+vm_length_1];
        elseif part == 3
            AllPeakTimes_final = [AllPeakTimes_final; AllPeakTimes+vm_length_1+vm_length_2];
        end       
    end
 
else
    % Open spike detection GUI
    % Returns AllPeakTimes
    spike_annotation_gui_V30(vm)
    AllPeakTimes_final = AllPeakTimes;
end

% Calculate spike events
AllPeakSamples = ceil(AllPeakTimes_final*sampling_rate);
AllPeakSamples(AllPeakSamples == 0) = 1;
length_vm = length(vm);

AllPeaks = zeros(length_vm, 1);
AllPeaks(AllPeakSamples) = 1;

spikes = AllPeaks;

% Save spikes 
save([file_path, file_name, '_spikes.mat'], 'spikes')
disp(['Data saved as ', file_path,file_name, '_spikes.mat'])