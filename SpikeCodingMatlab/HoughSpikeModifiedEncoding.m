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

function [spikes, shift] = HoughSpikeModifiedEncoding(input, fir, threshold)

    % Based on algorithm provided in:
    %   Schrauwen et al. (2003)
    
    L = length(input);
    F = length(fir);
    spikes = zeros(1,L);
    shift = min(input);
    input = input - shift;
    for i = 1:L
        error = 0;
        for j = 1:F
            if i+j-1 < L
                if input(i+j-1) < fir(j)
                    error = error + fir(j) - input(i+j-1);
                end
            end
        end
        if error <= threshold
            spikes(i) = 1;
            for j = 1:F
                if i+j-1 < L
                    input(i+j-1) = input(i+j-1) - fir(j);
                end
            end
        end
    end

end