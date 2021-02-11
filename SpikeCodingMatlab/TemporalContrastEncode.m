function [spikes, threshold] = TemporalContrastEncode(input, factor)

    % Based on algorithm provided in:
    %   Sengupta et al. (2017)
    %   Petro et al. (2020)
    
    L = length(input);
    diff = zeros(1,L-1);
    for i = 1:L-1
        diff(i) = input(i+1) - input(i);
    end 

    threshold = mean(diff) + factor*std(diff);
    diff = [diff(1), diff];

    spikes = zeros(1, L);
    for i = 1:L
        if diff(i) > threshold
            spikes(i) = 1;
        elseif diff(i) < -threshold
            spikes(i) = -1;
        end
    end    
    
end 

