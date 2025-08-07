function [x_vel, y_vel, z_vel, xy_speed, z_speed] = process_treadmill_data(x_vel, y_vel, z_vel, sampling_rate_ephys, sampling_rate_treadmill)
%PROCESS_TREADMILL_DATA processes treadmill data 
%
%   [x_vel, y_vel, z_vel, xy_speed, z_speed] = process_treadmill_data(x_vel, y_vel, z_vel, sampling_rate_ephys, sampling_rate_treadmill)
%   returns downsampled and filtered velocities and speeds from analog signals recorded with the treadmill.

% Author: Chris J. Dallmann 
% Affiliation: University of Wuerzburg
% Last revision: 07-August-2025

% ------------- BEGIN CODE -------------

% Convert to mm/s and deg/s
% Constants are setup-specific
x_vel = x_vel * 8.79; 
y_vel = y_vel * 8.79;
z_vel = z_vel * 158.9;

% Downsample 
x_vel = x_vel(1 : sampling_rate_ephys/sampling_rate_treadmill : end);
y_vel = y_vel(1 : sampling_rate_ephys/sampling_rate_treadmill : end);
z_vel = z_vel(1 : sampling_rate_ephys/sampling_rate_treadmill : end);
       
% Filter out baseline noise
filter_span = 0.3; % s
x_vel = smooth(x_vel, filter_span*sampling_rate_treadmill);
y_vel = smooth(y_vel, filter_span*sampling_rate_treadmill);
z_vel = smooth(z_vel, filter_span*sampling_rate_treadmill);

% Calculate translational speed
xy_speed = abs(x_vel) + abs(y_vel);

% Calculate rotational speed
z_speed = abs(z_vel);
