function x_filt = binary_hysteresis_filter(x,win_size)
%BINARY_HYSTERESIS_FILTER removes epochs of ones and zeros smaller than a given window size from a binary signal 
%
%   x_filt = BINARY_HYSTERESIS_FILTER(x,win_size) removes epochs of ones
%   and zeros smaller than win_size from binary signal x by assigning them
%   to the previous epoch.
%   
%   x        = vector of ones and zeros
%   win_size = window size  
% 
%   See also RunLength.m
% 
%   CJ Dallmann, University of Washington, 10/2021

% Example
%x = [1,1,1,0,1,1,0,0,1,0,1,1,1,1,0,0,0,1,0,1,0,0,0,0,1,0,0,0];
%win_size = 2;

% Find length of repeating elements (rep_elem) and their corresponding
% indices (idx)
[~,rep_elem,idx] = RunLength(x);

match = (rep_elem >= win_size);
start_idx = idx(match);

x_filt = nan(numel(x),1);
x_filt(1:start_idx(1)) = x(start_idx(1));
for iIdx = 1:numel(start_idx)-1
    x_filt(start_idx(iIdx):start_idx(iIdx+1)) = x(start_idx(iIdx));
end
x_filt(start_idx(end):end) = x(start_idx(end));

end