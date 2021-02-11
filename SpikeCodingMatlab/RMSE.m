function rmse = RMSE(data, estimate)

    rmse = sqrt( sum( (data(:)-estimate(:)).^2) / numel(data) );
     
end 