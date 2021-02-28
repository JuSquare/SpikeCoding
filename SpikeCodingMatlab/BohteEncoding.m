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

function spikes = BohteEncoding(input,m,beta)

    % Based on algorithm provided in:
    %   Boht� et al. (2002)
    
    % TODO: add temporal coding (not sure about what he meant in his paper)
    
    L = length(input);
    
    spikes = zeros(L,m);
    neuron_outputs = zeros(1,m);
    
    min_input = min(input);
    max_input = max(input);
    
    for j = 1:L
        for i = 1:m
           mu = min_input + (2*i-3)/2*(max_input - min_input)/(m-2);
           sigma = (m-2) / (beta * (max_input - min_input));
           neuron_outputs(i) = normpdf(input(j), mu, sigma)/normpdf(mu, mu, sigma);
        end
        [~,spikingNeuron] = max(neuron_outputs);
        spikes(j,spikingNeuron) = 1;
    end
    

end