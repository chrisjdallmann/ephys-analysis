function x_filt = binary_replace_filter(x,win_size)
%BINARY_REPLACE_FILTER replaces epochs of zeros smaller than a given window size with ones  
%
%   x_filt = BINARY_REPLACE_FILTER(x,win_size) replaces epochs of zeros
%   smaller than win_size with ones. 
%   
%   x        = vector of ones and zeros
%   win_size = window size  
% 
%   See also RunLength.m
% 
%   CJ Dallmann, University of Wuerzburg, 02/2025

% Example
%x = [1,1,1,0,1,1,0,0,1,0,1,1,1,1,0,0,0,1,0,1,0,0,0,0,1,0,0,0];
%win_size = 2;

% Find length of repeating elements (rep_elem) and their corresponding
% indices (idx)
[~,rep_elem,idx] = RunLength(x);

match = (rep_elem <= win_size);
start_idx = idx(match);
start_idx(x(start_idx)==1) = [];

x_filt = x;
for iIdx = 1:numel(start_idx)
    stop_idx = find(x(start_idx(iIdx):end)>0,1,'first') + start_idx(iIdx)-1;
    if stop_idx > numel(x)
        stop_idx = numel(x);
    end
    x_filt(start_idx(iIdx):stop_idx) = 1;
end

end