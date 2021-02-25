function spikes = BohteEncoding(input,m,beta)

    % Based on algorithm provided in:
    %   Bohté et al. (2002)
    
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