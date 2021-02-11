function [spikes, startpoint] = StepForwardEncoding(input, threshold)

    % Based on algorithm provided in:
    %   Petro et al. (2020)

    startpoint = input(1);
    L = length(input);
    spikes = zeros(1,L);
    base = startpoint;
    
    for i = 2:L
        if input(i) > base + threshold
            spikes(i) = 1;
            base = base + threshold;
        elseif input(i) < base - threshold
            spikes(i) = -1;
            base = base - threshold;
        end
    end
    
end