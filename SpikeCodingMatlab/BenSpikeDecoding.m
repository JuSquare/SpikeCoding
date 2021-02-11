function signal = BenSpikeDecoding(spikes, fir, shift)

    % Based on algorithm provided in:
    %   Petro et al. (2020)
    %   Sengupta et al. (2017)
    %   Schrauwen et al. (2003)
    
    signal = conv(spikes,fir) + shift;
    signal = signal(1:end-length(fir)+1);

end