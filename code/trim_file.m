% TRIM_FILE.m loads and trims abf-files
%
% Functions/toolboxes required:
%    load_abf.m

% Author: Chris J. Dallmann 
% Affiliation: University of Wuerzburg
% Last revision: 08-August-2025

% ------------- BEGIN CODE -------------

clear
clc

% Settings
file_name = '2025_05_09_0013';
file_path = 'Z:\Transfer\Chris\von Sirin\RRpushing\'; 
is_camera_trigger = true;
is_annotation = true;
annotations_path = 'Z:\Transfer\Chris\von Sirin\RRpushing\pushing_events.xlsx'; 

% Load file
[data, meta_data] = load_abf([file_path, file_name, '.abf']);

n_frames = size(data,1);
n_channels = size(data,2);

% Display file name
disp(file_name)

if ~is_camera_trigger
    % Plot data
    fig = figure();
    hold on
    for channel = 1:n_channels
        ax(channel) = subplot(n_channels, 1, channel);
        plot(data(:,channel),'k');
        set(gca,'Color','none')
        if channel<n_channels
            set(gca,'xticklabel','')
        end
    end
    linkaxes(ax,'x')
    hold off
    
    % Ask user for start and end frame
    start_frame_user = input('Enter start frame: ');
    end_frame_user = input('Enter end frame: ');
    
    if isempty(start_frame_user)
        start_frame = 1;
    end
    
    if isempty(end_frame_user)
        end_frame = n_frames;
    end

else
    % Get camera trigger data
    trigger_channel = find(strcmp(meta_data.recChNames, 'Cam_Trig'));
    trigger = data(:,trigger_channel);

    % Get indices of camera frames (rising edge)
    camera_frame_indices = peakfinder(diff(trigger), [], 0.5);
    camera_frame_indices = camera_frame_indices+1;

    % Set start and end frame
    if ~is_annotation
        start_frame = camera_frame_indices(1);
    else
        % Load annotations
        annotations = readtable(annotations_path);
        
        % Read start frame of video
        start_frame_video = annotations.start_frame(find(strcmp(annotations.file,file_name),1,'first'));

        % Get index of start_frame_video
        start_frame = camera_frame_indices(start_frame_video);
    end
    end_frame = camera_frame_indices(end);

    % Generate time vector
    %time = linspace(0,numel(trigger)/20000,numel(trigger))';
    %time = time-time(start_frame);
    %time = time-time(camera_frame_indices(1));
    time = 1:n_frames;

    % Plot data
    fig = figure();
    for channel = 1:n_channels
        ax(channel) = subplot(n_channels, 1, channel);
        hold on
        plot(time,data(:,channel),'k');
        % Indiciate start frame
        plot([time(start_frame), time(start_frame)], [min(data(:,channel)), max(data(:,channel))], 'm')
        plot([time(end_frame), time(end_frame)], [min(data(:,channel)), max(data(:,channel))], 'm')
        if channel == trigger_channel
            % Plot detected frames
            plot(time(camera_frame_indices), data(camera_frame_indices,channel), '.m')
        end
        hold off
        set(gca,'Color','none')
        if channel<n_channels
            set(gca,'xticklabel','')
        end
    end
    linkaxes(ax,'x')
    hold off

    % Ask user to correct start and end frame if necessary
    start_frame_user = input('Corrected start frame: ');
    end_frame_user = input('Corrected end frame: ');
    
    if ~isempty(start_frame_user)
        start_frame = start_frame_user;
    end
    
    if ~isempty(end_frame_user)
        end_frame = end_frame_user;
    end

end

% Display start and end frame
disp(['Start frame: ', num2str(start_frame)])
disp(['End frame: ', num2str(end_frame)])

% Trim data
data = data(start_frame:end_frame,:);
 
% % Save trimmed data
% disp('Saving data...')
% save([file_path, file_name, '.mat'], 'data')
% disp(['Data saved as ', file_path, file_name, '.mat'])