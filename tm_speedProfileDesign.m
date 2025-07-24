function [p_t, p_v] = tm_speedProfileDesign(v, t, f)
%   The function generates a triangular wave speed profile.
%
%   Input:
%   v = [v1, v2, v3], vector, represent the first, middle, and last speed
%   value.
%
%   t = [t1, t2, t3], vector, represent the first, middle, and last time
%   duration.
%
%   f, positive scalar, decides the sampling rate for the profile.
%
%   Output£º
%   p_t and p_v are both vectors, representing the time [s] (x-axis) and
%   speed command [m/s] (y-axis).
%
%   For example:
%   [p_t, p_v] = tm_speedProfileDesign([5, 0.6, 5], [2, 1, 2], 100);
%       Stage #1: speed is set as 2 m/s and last for 5 s.
%
%       Stage #2: peak point of the triangular speed is set as 1 m/s. The
%       time durations of the upward and downward slopes are 0.3 s each (0.6/2).
%
%       Stage #3: speed is set as 2 m/s and last for 5 s.
%
%       Sampling rate from the speed profile is 100 Hz.
%

if nargin < 3 || f <= 0
    p_t = 0;
    p_v = 0;
    return;
end

%% Profile value
% Stage #1
p_v     = v(1)*ones(round(t(1)*f), 1);

% Stage #2
if t(2) == 0
    p_v     = [p_v; v(3)*ones(round(t(3)*f), 1)];
else
    step    = (v(2)-v(1)) / (0.5*t(2)*f);
    slp_up  = v(1)+step:step:v(2);
    step    = (v(3)-v(2)) / (0.5*t(2)*f);
    slp_dn  = v(2)+step:step:v(3);
    
    % Stage #3
    p_v     = [p_v; slp_up'; slp_dn'; v(3)*ones(round(t(3)*f), 1)];
end

%% Profile time
p_t = (1/f:1/f:sum(t))';

%% plot
% figure; hold on; plot(p_t, p_v);

end

