function spikes_im = spikePlot(spikes)

    A = size(spikes);
    spikes_im = zeros(A(1), A(2), 3);
    for i = 1:A(1)
        for j = 1:A(2)
            if spikes(i,j,1) > 0
                spikes_im(i,j,3) = 255;
            elseif spikes(i,j,1) < 0
                spikes_im(i,j,1) = 255;
            end
        end
    end                

end