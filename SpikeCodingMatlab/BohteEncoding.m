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

function [spikes, min_input, max_input] = BohteEncoding(input,m,timesteps, beta)

    % Based on algorithm provided in:
    %   Bohté et al. (2002)
    
    L = length(input);
    
    spikes = zeros(L, timesteps, m);
    responses = zeros(m, 1);
        
    min_input = min(input);
    max_input = max(input);
        
    % Calculation of mu and sigma of the Gaussian receptive fields
    mrange = 1:m;
    mu = min_input + (2*mrange-3)/2*(max_input - min_input)/(m-2);
    sigma = 1/beta*(max_input - min_input)/(m-2);
    max_prob = normpdf(mu(1), mu(1), sigma);
    size_change = max_prob / (2 * timesteps);
    
    for j = 1:L
        for i = 1:m
            responses(i) = normpdf(input(j), mu(i), sigma);
            spike = round(((responses(i) + size_change) / (max_prob + 2 * size_change) * (timesteps + 1)) - 0.0001);
            spikeTime = timesteps - spike + 1;
            if spikeTime < timesteps - 1
                spikes(j, spikeTime, i) = 1;
            end
        end
    end
end