% POOL_DATA.m pools electrophysiology and behavior data across trials

% Author: Chris J. Dallmann 
% Affiliation: University of Wuerzburg
% Last revision: 14-August-2025

% ------------- BEGIN CODE -------------

clear
clc

% Settings
file_path = 'Z:\Transfer\Chris\von Sirin\RRflight\'; 
save_name = 'treadmill_ephys_rr_gfp_flight';
meta_data = readtable([file_path,'metadata.csv']);

% Initialize variables
sampling_rate_ephys = 20000;
sampling_rate_treadmill = 50;
experiments = meta_data.experiment;

% Loop over experiments
for experiment = 1:numel(experiments)
    % Get metadata
    file_name = experiments{experiment};

    % Load data
    load([file_path, file_name, '.mat']);
    [~, abf_meta_data] = load_abf([file_path, file_name, '.abf']);
    load([file_path, file_name, '_spikes.mat']);
    load([file_path, file_name, '_movement.mat']);

    % Get electrophysiology data
    membrane_potential = smooth(data(:,1), 10);
    spike_events = find(spikes>0);
    spike_rate = compute_spike_rate(spikes, sampling_rate_ephys, 0.15);

    % Create time vector
    n_frames_ephys = numel(membrane_potential);
    time_ephys = linspace(0, n_frames_ephys/sampling_rate_ephys, n_frames_ephys)';

    % Store data
    pooled_data(experiment).experiment = file_name;
    pooled_data(experiment).animal_id = meta_data.animal_id(experiment);
    pooled_data(experiment).trial = meta_data.trial(experiment);
    pooled_data(experiment).hemisphere = meta_data.hemisphere(experiment);
    pooled_data(experiment).membrane_potential = membrane_potential; 
    pooled_data(experiment).spike_events = spike_events;
    pooled_data(experiment).spike_rate = spike_rate;
    pooled_data(experiment).time_ephys = time_ephys;
    
    pooled_data(experiment).movement = movement;
    
    if contains(save_name,'walking')
        % Get treadmill data
        x_channel = find(strcmp(abf_meta_data.recChNames, 'Veloc X'));
        y_channel = find(strcmp(abf_meta_data.recChNames, 'Veloc Y'));
        z_channel = find(strcmp(abf_meta_data.recChNames, 'Veloc Z'));
        x_vel = data(:,x_channel);
        y_vel = data(:,y_channel);
        z_vel = data(:,z_channel);
        
        % Process treadmill data
        [x_vel, y_vel, z_vel, xy_speed, z_speed] = process_treadmill_data(...
            x_vel, y_vel, z_vel, sampling_rate_ephys, sampling_rate_treadmill);
        n_frames_treadmill = numel(x_vel);
        time_treadmill = linspace(0, n_frames_treadmill/sampling_rate_treadmill, n_frames_treadmill);

        pooled_data(experiment).forward_velocity = x_vel;
        pooled_data(experiment).lateral_velocity = y_vel;
        pooled_data(experiment).angular_velocity = z_vel;
        pooled_data(experiment).translational_speed = xy_speed;
        pooled_data(experiment).angular_speed = z_speed;
        pooled_data(experiment).time_treadmill = time_treadmill;
    end

    if contains(save_name,'flight')
        % Get tachometer data
        channel = find(strcmp(abf_meta_data.recChNames, 'Wingbeat_'));
        tachometer = data(:,channel);

        pooled_data(experiment).tachometer = tachometer;
    end
end

% Save pooled data
data = pooled_data;
save([file_path, save_name, '.mat'], 'data', '-v7.3')
disp(['Data saved as ', file_path, save_name, '.mat'])
