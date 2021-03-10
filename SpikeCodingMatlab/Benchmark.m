% This file is part of the SpikeCoding repository - MAVLab TU Delft
%
% MIT License
%
% Copyright (c) 2021 Julien Dupeyroux
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
%
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
%
% @author Julien Dupeyroux 

%% Get some data

% In this section, we design a benchmark of the 7 implemented methods based
% on a 1d signal (i.e., a sum of sine waves, including some noise). The
% benchmark includes a series of 'nb_tests' trials for which we determine,
% for each algorithm, the output spikes and the reconstructed signal. 

% We assess the sensitivity of the algorithms to long-term drift by setting
% the duration 'tMax' of the signal higher than one period. 

% For each trial, and for each algorithm, we determine the number of spikes
% output and store it in a matrix 'spike_count'. This is only done for the
% temporal and rate based methods. Indeed, in population coding, a spike is
% always emmited at each timestep. 

tMax = 1;
dt = 0.01;
T = dt:dt:tMax;

nb_tests = 1000;
S = zeros(nb_tests, length(T));

% Temporal contrast params
factor_TC = 0.5;
signal_TBR = zeros(nb_tests, length(T));

% Moving window params
threshold_MW = 0.25;
window_MW = 5;
signal_MW = zeros(nb_tests, length(T));

% Step forward params
threshold_SF = 0.5;
signal_SF = zeros(nb_tests, length(T));

% BSA params
fir_BSA = triang(9)';
threshold_BSA = 1.15;
signal_BSA = zeros(nb_tests, length(T));

% HSA params
fir_HSA = triang(15)';
signal_HSA = zeros(nb_tests, length(T));

% Threshold HSA params
fir_HSAm = triang(15)';
threshold_HSAm = 0.85;
signal_HSAm = zeros(nb_tests, length(T));

% Gaussian receptive fields params
nb_neurons_GRF = 20;
signal_GRF = zeros(nb_tests, length(T));

% Bohte params
nb_neurons_BOHTE = 15;
nb_timesteps_BOHTE = 10;
beta_BOHTE = 1.5;
signal_BOHTE = zeros(nb_tests, length(T));

% Spike counting for each of the methods (temporal, rate)
spike_count = zeros(nb_tests, 8);

for i = 1:nb_tests
    % Define the signal
    S(i,:) = 2*sin(2*pi*T) - 0.5*cos(0.5*pi*T) + 0.75*sin(10*pi*T) - 0.5*rand(1,length(T));
    
    % Temporal contrast
    [spikes_TBR, threshold] = TemporalContrastEncode(S(i,:),factor_TC);
    signal_TBR(i,:) = 2*TemporalContrastDecode(spikes_TBR, threshold);
    spike_count(i,1) = sum(abs(spikes_TBR));
    
    % Moving window
    [spikes_MW, startpoint] = MovingWindowEncoding(S(i,:), threshold_MW, window_MW);
    signal_MW(i,:) = MovingWindowDecoding(spikes_MW, threshold_MW, startpoint);
    spike_count(i,2) = sum(abs(spikes_MW));
    
    % Step forward
    [spikes_SF, startpoint] = StepForwardEncoding(S(i,:), threshold_SF);
    signal_SF(i,:) = StepForwardDecoding(spikes_SF, threshold_SF, startpoint);
    spike_count(i,3) = sum(abs(spikes_SF));
    
    % Ben spike
    [spikes_BSA, shift] = BenSpikeEncoding(S(i,:), fir_BSA, threshold_BSA);
    signal_BSA(i,:) = BenSpikeDecoding(spikes_BSA, fir_BSA, shift);
    spike_count(i,4) = sum(spikes_BSA);
    
    % Hough spike
    [spikes_HSA, shift] = HoughSpikeEncoding(S(i,:), fir_HSA);
    signal_HSA(i,:) = BenSpikeDecoding(spikes_HSA, fir_HSA, shift);
    spike_count(i,5) = sum(spikes_HSA);
    
    % Threshold Hough spike
    [spikes_HSAm, shift] = HoughSpikeModifiedEncoding(S(i,:), fir_HSAm, threshold_HSAm);
    signal_HSAm(i,:) = BenSpikeDecoding(spikes_HSAm, fir_HSAm, shift);
    spike_count(i,6) = sum(spikes_HSAm);
    
    % Gaussian field (population coding)
    [spikes_GRF,min_input,max_input] = GaussianReceptFieldsEncoding(S(i,:),nb_neurons_GRF);
    signal_GRF(i,:) = GaussianReceptFieldsDecoding(spikes_GRF,min_input,max_input);
    spike_count(i,7) = sum(spikes_GRF, 'all');
    
    % Bohte spike (temporal gaussian field coding)
    [spikes_BOHTE,min_input,max_input] = BohteEncoding(S(i,:),nb_neurons_BOHTE, nb_timesteps_BOHTE, beta_BOHTE);
    signal_BOHTE(i,:) = BohteDecoding(spikes_BOHTE,min_input,max_input);
    spike_count(i,8) = sum(spikes_BOHTE, 'all');
end

%% Plot results

figure()

subplot(6,3,[1,4])
hold on;
plot(T, signal_TBR', 'red')
plot(T, S', 'blue')
hold off;
title('Temporal Contrast Algorithm TBR');
box on;
subplot(6,3,7)
stem(T,spikes_TBR)
box on;

subplot(6,3,[2,5])
hold on;
plot(T, signal_MW', 'red')
plot(T, S', 'blue')
hold off;
title('Moving Window Algorithm MW');
box on;
subplot(6,3,8)
stem(T,spikes_MW)
box on;

subplot(6,3,[3,6])
hold on;
plot(T, signal_SF', 'red')
plot(T, S', 'blue')
hold off;
title('Step Forward Algorithm SF');
box on;
subplot(6,3,9)
stem(T,spikes_SF)
box on;

subplot(6,3,[10,13])
hold on;
plot(T, signal_BSA', 'red')
plot(T, S', 'blue')
hold off;
title('Ben Spike Algorithm BSA');
box on;
subplot(6,3,16)
stem(T,spikes_BSA)
box on;

subplot(6,3,[11,14])
hold on;
plot(T, signal_HSA', 'red')
plot(T, S', 'blue')
hold off;
title('Hough Spike Algorithm HSA');
box on;
subplot(6,3,17)
stem(T,spikes_HSA)
box on;

subplot(6,3,[12,15])
hold on;
plot(T, signal_HSAm', 'red')
plot(T, S', 'blue')
hold off;
title('Threshold Hough Spike Algorithm T-HSA');
box on;
subplot(6,3,18)
stem(T,spikes_HSAm)
box on;

figure()
subplot(3,1,[1,2])
hold on;
plot(T, signal_GRF', 'red')
plot(T, S', 'blue')
hold off;
title('Gaussian Receptive Fields Algorithm GRF');
box on;
subplot(3,1,3)
hold  all;
for  t = 1:length(T)
    [~,neuron] = max(spikes_GRF(t,:));
    plot([T(t) T(t)], [neuron-0.35 neuron+0.35], 'black', 'linewidth', 1.1);
end
hold off;
box on;

subplot(3,2,[1,3])
hold on;
plot(T, signal_GRF', 'red')
plot(T, S', 'blue')
hold off;
title('Bohté et al. (2002) GRF');
box on;
subplot(3,2,5)
hold  all;
for  t = 1:length(T)
    [~,neuron] = max(spikes_GRF(t,:));
    plot([T(t) T(t)], [neuron-0.35 neuron+0.35], 'black', 'linewidth', 1.1);
end
hold off;
box on;

subplot(3,2,[2,4])
hold on;
plot(T, signal_BOHTE', 'red')
plot(T, S', 'blue')
hold off;
title('Bohté et al. (2002) GRF');
box on;
subplot(3,2,6)
hold  all;
for  t = 1:length(T)
    [~,neuron] = max(spikes_BOHTE(t,:));
    plot([T(t) T(t)], [neuron-0.35 neuron+0.35], 'black', 'linewidth', 1.1);
end
hold off;
box on;

%% Statistical results

% We define the spike efficiency of an algorithm as ratio between the
% number of spikes emmitted to encode the signal, and the total number of
% spikes that could be emmitted (i.e., the length of the signal). 
% In this respect, we have:
%                                 spike_count
%       spike efficiency = ( 1 - ------------- ) x 100
%                                  length(T)


% note that this is not correct for the Bohte GRF, since this has a higher 
% amount of timesteps, and multiple neurons can fire at the same time.
spike_efficiency = (1 - spike_count/length(T))*100;


% boxplot(spike_efficiency, ...
%     'Labels', {'TBR', 'MW', 'SF', 'BSA', 'HSA', 'T-HSA'}, ...
%     'Symbol', 'bo')

means_efficiency = mean(spike_efficiency)
median_efficiency = median(spike_efficiency)
sd_efficiency = std(spike_efficiency)

figure()
plot(spike_efficiency(:, 1:6))
legend('TBR','MW','SF','BSA','HSA','T-HSA') 
axis([1 nb_tests 0 100])

% We also look at the root mean square error between the original signal
% and the reconstructed version. 

rmse = zeros(nb_tests, 6);
for i = 1:nb_tests
    rmse(i,1) = RMSE(S(i,:), signal_TBR(i,:));
    rmse(i,2) = RMSE(S(i,:), signal_MW(i,:));
    rmse(i,3) = RMSE(S(i,:), signal_SF(i,:));
    rmse(i,4) = RMSE(S(i,:), signal_BSA(i,:));
    rmse(i,5) = RMSE(S(i,:), signal_HSA(i,:));
    rmse(i,6) = RMSE(S(i,:), signal_HSAm(i,:));
    rmse(i,7) = RMSE(S(i,:), signal_GRF(i,:));
    rmse(i,8) = RMSE(S(i,:), signal_BOHTE(i,:));
end

means_rmse = mean(rmse)
median_rmse = median(rmse)
sd_rmse = std(rmse)

figure()
plot(rmse)
legend('TBR','MW','SF','BSA','HSA','T-HSA', 'GRF', 'BOHTE')
axis([1 nb_tests 0 2])

%% Results

% Spiking efficiency 
%   -----------------------------------------------------------------------
%   Method         TBR         MW       SF        BSA       HSA      T-HSA
%   -----------------------------------------------------------------------
%   - Case 0: T_max = 1 sec (1 period), nb_tests = 1000
%       means   = 36.6670   29.1840   74.5520   45.8340   73.9570   62.3660
%       medians = 37.0000   29.0000   74.0000   46.0000   74.0000   62.0000
%       sd      = 4.1023    3.3852    2.0331    1.7792    1.3368    1.1159
%   - Case 1: T_max = 5 sec (5 periods), nb_tests = 1000
%       means   = 36.7118   27.4760   73.4424   33.0960   64.4994   57.4648
%       medians = 36.8000   27.4000   73.6000   33.0000   64.4000   57.4000
%       sd      = 1.7279    1.5102    0.8752    1.2934    1.0042    1.0881
%   -----------------------------------------------------------------------
%   - Case 2: T_max = 15 sec (15 periods), nb_tests = 1000
%       means   = 36.6441   27.2098   73.2322   29.9533   62.0691   55.7606
%       medians = 36.6667   27.2667   73.2000   29.8000   62.0000   55.6000
%       sd      = 1.0202    0.8785    0.5133    0.7859    0.6049    0.6289
%   -----------------------------------------------------------------------
%   - Case 3: T_max = 50 sec (50 periods), nb_tests = 1000
%       means   = 36.6582   27.1176   73.1904   29.2558   61.6128   55.5813
%       medians = 36.6600   27.1200   73.1800   29.2000   61.5800   55.5400
%       sd      = 0.5641    0.4609    0.3193    0.3782    0.2808    0.2842
%   -----------------------------------------------------------------------
%   - Case 4: T_max = 100 sec (100 periods), nb_tests = 1000
%       means   = 36.6285   27.0948   73.1889   28.9774   61.4006   55.4502
%       medians = 36.6200   27.1000   73.1900   28.9300   61.3700   55.4100
%       sd      = 0.3972    0.3398    0.2390    0.2453    0.1844    0.1782
%   -----------------------------------------------------------------------

% RMSE
%   -----------------------------------------------------------------------
%   Method         TBR         MW       SF        BSA       HSA      T-HSA
%   -----------------------------------------------------------------------
%   - Case 0: T_max = 1 sec (1 period), nb_tests = 1000
%       means   = 0.7966    0.6764    0.2597    0.8387    0.9669    0.6778
%       medians = 0.7221    0.6147    0.2606    0.8404    0.9650    0.6785
%       sd      = 0.3860    0.2362    0.0147    0.0375    0.0451    0.0275
%   - Case 1: T_max = 5 sec (5 periods), nb_tests = 1000
%       means   = 1.4100    1.1092    0.2624    0.6392    0.6178    0.3919
%       medians = 1.2278    0.9651    0.2620    0.6388    0.6179    0.3922
%       sd      = 0.6828    0.5381    0.0066    0.0166    0.0277    0.0110
%   -----------------------------------------------------------------------
%   - Case 2: T_max = 15 sec (15 periods), nb_tests = 1000
%       means   = 2.2356    1.7386    0.2624    0.5970    0.5147    0.3011
%       medians = 1.9528    1.5297    0.2625    0.5973    0.5149    0.3010
%       sd      = 1.1664    0.8620    0.0040    0.0098    0.0177    0.0059
%   -----------------------------------------------------------------------
%   - Case 3: T_max = 50 sec (50 periods), nb_tests = 1000
%       means   = 3.8957    2.9421    0.2626    0.5801    0.4752    0.2612
%       medians = 3.3912    2.5364    0.2625    0.5802    0.4753    0.2611
%       sd      = 2.0055    1.5680    0.0022    0.0048    0.0101    0.0035
%   -----------------------------------------------------------------------
%   - Case 4: T_max = 100 sec (100 periods), nb_tests = 1000
%       means   = 5.5450    4.1396    0.2626    0.5739    0.4606    0.2516
%       medians = 4.6629    3.6058    0.2626    0.5739    0.4608    0.2516
%       sd      = 2.9807    2.1694    0.0015    0.0032    0.0075    0.0026
%   -----------------------------------------------------------------------
