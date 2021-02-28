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

%% The 1-D signals for testing

tMax = 1;
dt = 0.01;
T = dt:dt:tMax;

S1 = 2*sin(2*pi*T) - 0.5*cos(6*pi*T) + 0.75*sin(10*pi*T);
S2 = -0.25*sin(2*pi*T) - 0.05*rand(1,length(T));
S3 = normpdf(T,0.2,0.1) + 0.5*normpdf(T,0.75,0.1) - 0.1*rand(1,length(T));

%% Comparing TBR, SF and MW

figure()

factor = 1.005;
[spikes_TBR_1, threshold] = TemporalContrastEncode(S1,factor);
signal_TBR_1 = 2*TemporalContrastDecode(spikes_TBR_1, threshold);
factor = 1.005;
[spikes_TBR_2, threshold] = TemporalContrastEncode(S2,factor);
signal_TBR_2 = 2*TemporalContrastDecode(spikes_TBR_2, threshold);
factor = 1.005;
[spikes_TBR_3, threshold] = TemporalContrastEncode(S3,factor);
signal_TBR_3 = 2*TemporalContrastDecode(spikes_TBR_3, threshold);

subplot(9,3,[1,4])
hold on;
plot(T,S1)
plot(T,signal_TBR_1)
hold off;
title('Temporal Contrast Algorithm TBR');
box on;
subplot(9,3,7)
stem(T,spikes_TBR_1)
box on;

subplot(9,3,[10,13])
hold on;
plot(T,S2)
plot(T,signal_TBR_2)
hold off;
box on;
subplot(9,3,16)
stem(T,spikes_TBR_2)
box on;

subplot(9,3,[19,22])
hold on;
plot(T,S3)
plot(T,signal_TBR_3)
hold off;
box on;
subplot(9,3,25)
stem(T,spikes_TBR_3)
box on;

threshold = 0.35;
[spikes_SF_1, startpoint] = StepForwardEncoding(S1, threshold);
signal_SF_1 = StepForwardDecoding(spikes_SF_1, threshold, startpoint);
threshold = 0.05;
[spikes_SF_2, startpoint] = StepForwardEncoding(S2, threshold);
signal_SF_2 = StepForwardDecoding(spikes_SF_2, threshold, startpoint);
threshold = 0.35;
[spikes_SF_3, startpoint] = StepForwardEncoding(S3, threshold);
signal_SF_3 = StepForwardDecoding(spikes_SF_3, threshold, startpoint);

subplot(9,3,[2,5])
hold on;
plot(T,S1)
plot(T,signal_SF_1)
hold off;
title('Step Forward Algorithm SF');
box on;
subplot(9,3,8)
stem(T,spikes_SF_1)
box on;

subplot(9,3,[11,14])
hold on;
plot(T,S2)
plot(T,signal_SF_2)
hold off;
box on;
subplot(9,3,17)
stem(T,spikes_SF_2)
box on;

subplot(9,3,[20,23])
hold on;
plot(T,S3)
plot(T,signal_SF_3)
hold off;
box on;
subplot(9,3,26)
stem(T,spikes_SF_3)
box on;

threshold = 0.325;
window = 3;
[spikes_MW_1, startpoint] = MovingWindowEncoding(S1, threshold, window);
signal_MW_1 = MovingWindowDecoding(spikes_MW_1, threshold, startpoint);
threshold = 0.015;
window = 3;
[spikes_MW_2, startpoint] = MovingWindowEncoding(S2, threshold, window);
signal_MW_2 = MovingWindowDecoding(spikes_MW_2, threshold, startpoint);
threshold = 0.225;
window = 3;
[spikes_MW_3, startpoint] = MovingWindowEncoding(S3, threshold, window);
signal_MW_3 = MovingWindowDecoding(spikes_MW_3, threshold, startpoint);

subplot(9,3,[3,6])
hold on;
plot(T,S1)
plot(T,signal_MW_1)
hold off;
title('Moving Window Algorithm MW');
box on;
subplot(9,3,9)
stem(T,spikes_MW_1)
box on;

subplot(9,3,[12,15])
hold on;
plot(T,S2)
plot(T,signal_MW_2)
hold off;
box on;
subplot(9,3,18)
stem(T,spikes_MW_2)
box on;

subplot(9,3,[21,24])
hold on;
plot(T,S3)
plot(T,signal_MW_3)
hold off;
box on;
subplot(9,3,27)
stem(T,spikes_MW_3)
box on;

%% Comparing HSA, HSA modified and BSA

figure()

W = 12;
fir = triang(W)';
[spikes_HSA_1, shift] = HoughSpikeEncoding(S1, fir);
signal_HSA_1 = BenSpikeDecoding(spikes_HSA_1, fir, shift);
W = 15;
fir = normpdf(1:W,0,5)';
[spikes_HSA_2, shift] = HoughSpikeEncoding(S2, fir);
signal_HSA_2 = BenSpikeDecoding(spikes_HSA_2, fir, shift);
W = 12;
fir = triang(W)';
[spikes_HSA_3, shift] = HoughSpikeEncoding(S3, fir);
signal_HSA_3 = BenSpikeDecoding(spikes_HSA_3, fir, shift);

subplot(9,3,[1,4])
hold on;
plot(T,S1)
plot(T,signal_HSA_1)
hold off;
title('Hough Spike Algorithm HSA');
box on;
subplot(9,3,7)
stem(T,spikes_HSA_1)
box on;

subplot(9,3,[10,13])
hold on;
plot(T,S2)
plot(T,signal_HSA_2)
hold off;
box on;
subplot(9,3,16)
stem(T,spikes_HSA_2)
box on;

subplot(9,3,[19,22])
hold on;
plot(T,S3)
plot(T,signal_HSA_3)
hold off;
box on;
subplot(9,3,25)
stem(T,spikes_HSA_3)
box on;

W = 12;
fir = triang(W)';
threshold = 0.85;
[spikes_HSAm_1, shift] = HoughSpikeModifiedEncoding(S1, fir, threshold);
signal_HSAm_1 = BenSpikeDecoding(spikes_HSAm_1, fir, shift);
W = 15;
fir = normpdf(1:W,0,5)';
threshold = 0.05;
[spikes_HSAm_2, shift] = HoughSpikeModifiedEncoding(S2, fir, threshold);
signal_HSAm_2 = BenSpikeDecoding(spikes_HSAm_2, fir, shift);
W = 12;
fir = triang(W)';
threshold = 0.5;
[spikes_HSAm_3, shift] = HoughSpikeModifiedEncoding(S3, fir, threshold);
signal_HSAm_3 = BenSpikeDecoding(spikes_HSAm_3, fir, shift);

subplot(9,3,[2,5])
hold on;
plot(T,S1)
plot(T,signal_HSAm_1)
hold off;
title('Threshold Hough Spike Algorithm T-HSA');
box on;
subplot(9,3,8)
stem(T,spikes_HSAm_1)
box on;

subplot(9,3,[11,14])
hold on;
plot(T,S2)
plot(T,signal_HSAm_2)
hold off;
box on;
subplot(9,3,17)
stem(T,spikes_HSAm_2)
box on;

subplot(9,3,[20,23])
hold on;
plot(T,S3)
plot(T,signal_HSAm_3)
hold off;
box on;
subplot(9,3,26)
stem(T,spikes_HSAm_3)
box on;

W = 9;
fir = triang(W)';
threshold = 1.175;
[spikes_BSA_1, shift] = BenSpikeEncoding(S1, fir, threshold);
signal_BSA_1 = BenSpikeDecoding(spikes_BSA_1, fir, shift);
W = 10;
fir = normpdf(1:W,1.5,3.5)';
threshold = 1.05;
[spikes_BSA_2, shift] = BenSpikeEncoding(S2, fir, threshold);
signal_BSA_2 = BenSpikeDecoding(spikes_BSA_2, fir, shift);
W = 8;
fir = triang(W)';
threshold = 1.2;
[spikes_BSA_3, shift] = BenSpikeEncoding(S3, fir, threshold);
signal_BSA_3 = BenSpikeDecoding(spikes_BSA_3, fir, shift);

subplot(9,3,[3,6])
hold on;
plot(T,S1)
plot(T,signal_BSA_1)
hold off;
title('Ben Spike Algorithm BSA');
box on;
subplot(9,3,9)
stem(T,spikes_BSA_1)
box on;

subplot(9,3,[12,15])
hold on;
plot(T,S2)
plot(T,signal_BSA_2)
hold off;
box on;
subplot(9,3,18)
stem(T,spikes_BSA_2)
box on;

subplot(9,3,[21,24])
hold on;
plot(T,S3)
plot(T,signal_BSA_3)
hold off;
box on;
subplot(9,3,27)
stem(T,spikes_BSA_3)
box on;

%% Plot Gaussians

% figure()
% x = -5:0.001:7.5;
% Nj = 10;
% 
% sgtitle('Gaussian receptive fields');
% hold on;
% for j = 1:Nj
%     signal = input(j*10);
%     for i = 1:m
%        mu = min_input + (2*i-3)/2*(max_input - min_input)/(m-2);
%        % Bohté et al. 2002: 
%        % sigma = (m-2) / (beta * (max_input - min_input));
%        % Modified version: 
%        sigma = (max_input - min_input)/(m-2);
%        plot(x, normpdf(x, mu, sigma)/normpdf(mu, mu, sigma));
%        plot(signal, normpdf(signal, mu, sigma)/normpdf(mu, mu, sigma), 'ob');
%        xline(signal,'--r');
%        xline(min_input, 'black', 'linewidth', 1.5);
%        xline(max_input, 'black', 'linewidth', 1.5);
%     end
% end
% box on;
% axis([min_input-3*sigma max_input+3*sigma 0 1]);


%% Comparing GFR (Linear), GFR (Gaussian), and Bohté et al. (2002)

figure()

m = 15;
[spikes_GFR_1,min_input,max_input] = GaussianReceptFieldsEncoding(S1,m);
signal_GFR_1 = GaussianReceptFieldsDecoding(spikes_GFR_1,min_input,max_input);
[spikes_GFR_2,min_input,max_input] = GaussianReceptFieldsEncoding(S2,m);
signal_GFR_2 = GaussianReceptFieldsDecoding(spikes_GFR_2,min_input,max_input);
[spikes_GFR_3,min_input,max_input] = GaussianReceptFieldsEncoding(S3,m);
signal_GFR_3 = GaussianReceptFieldsDecoding(spikes_GFR_3,min_input,max_input);

subplot(9,3,[1,4])
hold on;
plot(T,S1)
plot(T,signal_GFR_1)
hold off;
title('Gaussian Receptive Fields GRF');
box on;
subplot(9,3,7)
hold  all;
for  t = 1:length(T)
    [~,neuron] = max(spikes_GFR_1(t,:));
    plot([T(t) T(t)], [neuron-0.35 neuron+0.35], 'black', 'linewidth', 1.1);
end
hold off;
box on;

subplot(9,3,[10,13])
hold on;
plot(T,S2)
plot(T,signal_GFR_2)
hold off;
box on;
subplot(9,3,16)
hold  all;
for  t = 1:length(T)
    [~,neuron] = max(spikes_GFR_2(t,:));
    plot([T(t) T(t)], [neuron-0.35 neuron+0.35], 'black', 'linewidth', 1.1);
end
hold off;
box on;

subplot(9,3,[19,22])
hold on;
plot(T,S3)
plot(T,signal_GFR_3)
hold off;
box on;
subplot(9,3,25)
hold  all;
for  t = 1:length(T)
    [~,neuron] = max(spikes_GFR_3(t,:));
    plot([T(t) T(t)], [neuron-0.35 neuron+0.35], 'black', 'linewidth', 1.1);
end
hold off;
box on;

