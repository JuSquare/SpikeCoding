% This file is part of the SpikeCoding repository - MAVLab TU Delft
%
% MIT License
%
% Copyright (c) 2021 Julien Dupeyroux
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
%
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
%
% @author Julien Dupeyroux

function signal = MovingWindowDecoding(spikes, threshold, startpoint)

    % Based on algorithm provided in:
    %   Petro et al. (2020)
    
    L = length(spikes);
    signal = zeros(1,L);
    signal(1) = startpoint;
    
    for i = 2:L
        if spikes(i) > 0
            signal(i) = signal(i-1) + threshold;
        elseif spikes(i) < 0
            signal(i) = signal(i-1) - threshold;
        else
            signal(i) = signal(i-1);
        end
    end

end