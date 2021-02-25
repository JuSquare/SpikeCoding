function signal = GaussianReceptFieldsDecoding(spikes,min_input,max_input)

    L = size(spikes,1);
    m = size(spikes,2);
    signal = zeros(1,L);
    for i = 1:L
        [~,index_neuron] = max(spikes(i,:));
        signal(1,i) = min_input + (2*index_neuron-3)/2*(max_input - min_input)/(m-2);
    end
    

end