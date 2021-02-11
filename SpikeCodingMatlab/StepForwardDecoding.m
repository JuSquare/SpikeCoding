function signal = StepForwardDecoding(spikes, threshold, startpoint)

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