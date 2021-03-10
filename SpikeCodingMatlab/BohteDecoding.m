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
% @author Stein Stroobants

function signal = BohteDecoding(spikes, min_input, max_input)
    % Based on algorithm provided in:
    %   Bohté et al. (2002)

    shape = size(spikes);
    signal = zeros(shape(1), 1);
    mu = zeros(shape(3), 1);

    for i = 1:shape(3)
        mu(i) = min_input + (2*i-3)/2*(max_input - min_input)/(shape(3)-2);
    end
    
    for i = 1:shape(1)
        spike_times = zeros(shape(3), 1);
        for j = 1:shape(2)
            for spike_idx = find(spikes(i, j, :) > 0)
                spike_times(spike_idx) = shape(2) - j;
            end
        end
        weight_center  = sum(mu.*spike_times)/sum(spike_times);
        signal(i) = weight_center;
    end
end