function [spikes,min_input,max_input] = GaussianReceptFieldsEncoding(input,m)
    
    L = length(input);
    
    spikes = zeros(L,m);
    neuron_outputs = zeros(1,m);
    
    min_input = min(input);
    max_input = max(input);
    
    for j = 1:L
        for i = 1:m
           mu = min_input + (2*i-3)/2*(max_input - min_input)/(m-2);
           sigma = (max_input - min_input)/(m-2);
           neuron_outputs(i) = normpdf(input(j), mu, sigma)/normpdf(mu, mu, sigma);
        end
        [~,spikingNeuron] = max(neuron_outputs);
        spikes(j,spikingNeuron) = 1;
    end
    

end