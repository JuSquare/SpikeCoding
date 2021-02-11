function [spikes, shift] = HoughSpikeEncoding(input, fir)

    % Based on algorithm provided in:
    %   Schrauwen et al. (2003)
    
    L = length(input);
    F = length(fir);
    spikes = zeros(1,L);
    shift = min(input);
    input = input - shift;
    for i = 1:L
        count = 0;
        for j = 1:F
            if i+j-1 < L
                if input(i+j-1) >= fir(j)
                    count = count + 1;
                end
            end
        end
        if count == F
            spikes(i) = 1;
            for j = 1:F
                if i+j-1 < L
                    input(i+j-1) = input(i+j-1) - fir(j);
                end
            end
        end
    end

end