function [spikes, shift] = BenSpikeEncoding(input, fir, threshold)

    % Based on algorithm provided in:
    %   Petro et al. (2020)
    %   Sengupta et al. (2017)
    %   Schrauwen et al. (2003)
    
    L = length(input);
    F = length(fir);
    spikes = zeros(1,L);
    shift = min(input);
    input = input - shift;
    
    for i = 1:(L-F)
        err1 = 0;
        err2 = 0;
        for j = 1:F
            err1 = err1 + abs(input(i+j) - fir(j));
            err2 = err2 + abs(input(i+j-1));
        end
        if err1 <= err2*threshold
            spikes(i) = 1;
            for j = 1:F
                if i+j+1 <= L
                    input(i+j+1) = input(i+j+1) - fir(j);
                end
            end
        end
    end
    
end