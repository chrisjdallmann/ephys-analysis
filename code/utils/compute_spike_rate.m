function spike_rate = compute_spike_rate(spike_events, sampling_rate, kernel_width)
%COMPUTE_SPIKE_RATE estimates a continuous spike rate from spike times using a Gaussian kernel
% 
%   spike_rate = EPHYS_COMPUTE_SPIKE_RATE(spike_events, sampling_rate, kernel_width) 
%   estimates a continuous spike rate from spike times using Gaussian kernel smoothing
%   
%   spike_events   = binary vector with ones indicating spikes  
%   sampling_rate  = sampling rate in Hz 
%   kernel_width   = standard deviation of Gaussian kernel in seconds 
% 
%   CJ Dallmann, University of Wuerzburg, 07/2025

kernel_width = kernel_width * sampling_rate;

win_width = 4 * kernel_width * 2; % Extend window width to 4x standard deviation in each direction to avoid clipping of Gaussian  

alpha = (win_width-1) / (2*kernel_width);
w = gausswin(win_width, alpha);
w = w/sum(w);

spike_rate = conv(spike_events, w, 'same');
spike_rate = spike_rate * sampling_rate;

end