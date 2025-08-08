% DETECT_MOVEMENT.m detects walking, flight, and other movement bouts 
%
% Functions/toolboxes required:
%    process_treadmill_data.m

% Author: Chris J. Dallmann 
% Affiliation: University of Wuerzburg
% Last revision: 08-August-2025

% ------------- BEGIN CODE -------------

clear
clc

% Settings
file_name = '2025_07_04_0011';
file_path = 'Z:\Transfer\Chris\von Sirin\RRpushing\'; 
movement_type = 'Pushing'; 
annotations_path = 'Z:\Transfer\Chris\von Sirin\RRpushing\pushing_events_joint.xlsx'; 

sampling_rate_ephys = 20000;
sampling_rate_treadmill = 50;
sampling_rate_camera = 150;

% Load file
load([file_path, file_name, '.mat']);
[~, meta_data] = abfload_Sander([file_path, file_name, '.abf']);

n_frames_ephys = size(data,1);
n_frames_treadmill = round(n_frames_ephys / (sampling_rate_ephys/sampling_rate_treadmill));
n_frames_camera = round(n_frames_ephys / (sampling_rate_ephys/sampling_rate_camera));


% Detect movement
if strcmp(movement_type,'Walking')
    % Initialize movement vector
    movement = zeros(n_frames_treadmill,1);
    
    % Get treadmill data
    x_channel = find(strcmp(meta_data.recChNames, 'Veloc X'));
    y_channel = find(strcmp(meta_data.recChNames, 'Veloc Y'));
    z_channel = find(strcmp(meta_data.recChNames, 'Veloc Z'));
    x_vel = data(:,x_channel);
    y_vel = data(:,y_channel);
    z_vel = data(:,z_channel);
      
    % Process treadmill data
    [x_vel, y_vel, z_vel, xy_speed, z_speed] = process_treadmill_data(...
        x_vel, y_vel, z_vel, sampling_rate_ephys, sampling_rate_treadmill);

    % Detect frames where translational or rotational speed is above
    % threshold
    xy_speed_thresh = 0.3; % mm/s
    z_speed_thresh = 10; % deg/s
    
    xy_movement = movement;
    z_movement = movement;
    
    xy_movement(xy_speed > xy_speed_thresh) = 1;
    z_movement(z_speed > z_speed_thresh) = 1;
    
    movement(xy_movement==1 | z_movement==1) = 1;

    % Fill short gaps 
    win_size = 0.1; % s
    movement = binary_replace_filter(movement, win_size*sampling_rate_treadmill);
    
    % Filter out short bouts
    win_size = 0.3; % s
    movement_fwd = binary_hysteresis_filter(movement, win_size*sampling_rate_treadmill);
    movement_bwd = binary_hysteresis_filter(flipud(movement), win_size*sampling_rate_treadmill);
    movement = double(movement_fwd | flipud(movement_bwd));

    % Plot 
    figure
    ax1 = subplot(3,1,1);
        plot(movement, 'k')
        set(gca, 'Color', 'none', 'ylim', [-0.5, 1.5])
    ax2 = subplot(3,1,2);
        plot(xy_speed, 'k')
        set(gca, 'Color', 'none')
    ax3 = subplot(3,1,3);
        plot(z_speed, 'k')
        set(gca, 'Color', 'none')
    linkaxes([ax1,ax2,ax3], 'x')

    % Upsample to sampling rate of ephys
    upsampling_factor = sampling_rate_ephys/sampling_rate_treadmill;
    n = 1:length(movement);
    nq = linspace(1, length(movement), floor(length(movement)*upsampling_factor));
    movement_upsampled = interp1(n, movement, nq, 'nearest')';
    % Add missing frames at the end
    movement_upsampled = [movement_upsampled; ...
        repmat(movement_upsampled(end), n_frames_ephys-numel(movement_upsampled), 1)]; 

    movement = movement_upsampled;


elseif strcmp(movement_type,'Flight')
    % Initialize movement vector
    movement = zeros(n_frames_ephys,1);
       
    % Get tachometer data
    channel = find(strcmp(meta_data.recChNames, 'Wingbeat_'));
    tacho = data(:,channel);

    % Plot data
    figure
    plot(tacho, 'k')
    set(gca, 'Color', 'none', 'ylim', [-12, 12])
    
    % Ask user for threshold
    threshold = input('Enter threshold: ');

    % Detect peaks
    if threshold>0
        index = peakfinder(tacho, [], threshold);
    else
        index = peakfinder(tacho*-1, [], threshold*-1);
    end

    % Update movement vector
    movement(index) = 1;
    
    % Fill in gaps between peaks  
    win_size = 0.01; % s
    movement = binary_replace_filter(movement, win_size*sampling_rate_ephys);
    
    % Assign short epochs to previous epoch
    win_size = 0.3; % s
    movement_fwd = binary_hysteresis_filter(movement, win_size*sampling_rate_ephys);
    movement_bwd = binary_hysteresis_filter(flipud(movement), win_size*sampling_rate_ephys);
    movement = double(movement_fwd | flipud(movement_bwd));

    % Plot 
    figure
    ax1 = subplot(2,1,1);
        plot(movement, 'k')
        set(gca, 'Color', 'none', 'ylim', [-0.5, 1.5])
    ax2 = subplot(2,1,2);
        hold on
        plot(tacho, 'k')
        plot(index, tacho(index), '.m')
        hold off
        set(gca, 'Color', 'none', 'ylim', [-12, 12])
    linkaxes([ax1,ax2], 'x')


elseif strcmp(movement_type,'Pushing')
    % Initialize movement vector
    movement = zeros(n_frames_camera,1);
     
    % Load annotations
    annotations = readtable(annotations_path);
    annotations = annotations(strcmp(annotations.file,file_name), :);   

    % Loop over events
    n_events = size(annotations,1);
    start_frame = annotations.start_frame(1);
    for event = 1:n_events
        onset = annotations.onset(event) - start_frame + 1;
        offset = annotations.offset(event) - start_frame + 1;
        movement(onset:offset) = 1;
    end
    
    % Upsample to sampling rate of ephys
    upsampling_factor = sampling_rate_ephys/sampling_rate_camera;
    n = 1:length(movement);
    nq = linspace(1, length(movement), floor(length(movement)*upsampling_factor));
    movement_upsampled = interp1(n, movement, nq, 'nearest')';
    % Add missing frames at the end
    movement_upsampled = [movement_upsampled; ...
        repmat(movement_upsampled(end), n_frames_ephys-numel(movement_upsampled), 1)]; 

    % % Check upsampling
    % figure
    % subplot(211)
    %     plot(movement)
    %     xlim([1,length(movement)])
    % subplot(212)
    %     plot(movement_upsampled)
    %     xlim([1,length(movement_upsampled)])

    movement = movement_upsampled;

end

% Save movement data
save([file_path, file_name, '_movement.mat'], 'movement')
disp(['Data saved as ', file_path, file_name, '_movement.mat'])