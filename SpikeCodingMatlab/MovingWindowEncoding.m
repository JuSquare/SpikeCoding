function [spikes, startpoint] = MovingWindowEncoding(input, threshold, window)

    % Based on algorithm provided in:
    %   Petro et al. (2020)

    startpoint = input(1);
    L = length(input);
    spikes = zeros(1,L);
    base = mean(input(1:(window+1)));
    
    for i = 1:(window+1)
        if input(i) > base + threshold
            spikes(i) = 1;
        elseif input(i) < base - threshold
            spikes(i) = -1;
        end
    end
    
    for i = (window+2):L
        base = mean(input((i-window-1):(i-1)));
        if input(i) > base + threshold
            spikes(i) = 1;
        elseif input(i) < base - threshold
            spikes(i) = -1;
        end
    end
    
end