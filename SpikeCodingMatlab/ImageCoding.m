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

%% Load data

filename = 'visual_data/3/video.avi';
myVid = VideoReader(filename);
T = 5;
inputData = zeros(myVid.Height, myVid.Width, T);

for i=1:T
    frame = rgb2gray(readFrame(myVid));
    inputData(:,:,i) = frame;
end

Time = 0:(1/myVid.FrameRate):(T-1)/myVid.FrameRate;

%% Plots 

Pixel = [990, 190];
deltaPix = 25;

Signal = squeeze(inputData(Pixel(1),Pixel(2),:));

% Plot one image for example
figure()
subplot(5,1,1:4)
hold on;
imagesc(inputData(:,:,T));
rectangle('Position',[Pixel(1)-deltaPix Pixel(2)-deltaPix deltaPix*2 deltaPix*2], 'linewidth', 1.5);
plot(Pixel(1), Pixel(2), 'red+', 'linewidth', 1.5);
hold off;
box on;
axis([0 myVid.Width 0 myVid.Height])
colorbar

% Plot variations of a pixel
subplot(5,1,5)
plot(Time, Signal);
box on;
axis([0 Time(end) min(Signal) max(Signal)])
xlabel('Time [s]');
ylabel('Gray level');

%% Test Step-Forward Temporal Contrast algorighm on one pixel

% Get the signal
Pixel = [990, 190];
Signal = squeeze(inputData(Pixel(1),Pixel(2),:));

% Step forward params
threshold_SF = 3;

% Step forward coding
[spikes_SF, startpoint] = StepForwardEncoding(Signal, threshold_SF);
signal_SF = StepForwardDecoding(spikes_SF, threshold_SF, startpoint);

figure()
subplot(4,1,1:3)
hold on;
plot(Time, signal_SF, 'red')
plot(Time, Signal, 'blue')
hold off;
title('Step Forward Algorithm SF');
box on;
subplot(4,1,4)
stem(Time,spikes_SF)
box on;

%% Applying SF algorithm to the whole image

spikes = zeros(myVid.Height, myVid.Width, T);
reconstruct_images = zeros(myVid.Height, myVid.Width, T);
threshold_SF = 20;

for i = 1:myVid.Height
    for j = 1:myVid.Width
        [spikes(i,j,:), startpoint] = StepForwardEncoding(squeeze(inputData(i,j,:)), threshold_SF);
        reconstruct_images(i,j,:) = StepForwardDecoding(spikes(i,j,:), threshold_SF, startpoint);
    end
end

%% Plots

figure('Position', [10 10 2400 600])
subplot(1,3,1)
imshow(inputData(:,:,T),[]);
title('Input');
axis equal;
axis([0 myVid.Width 0 myVid.Height])
subplot(1,3,2)
imshow(spikePlot(spikes(:,:,T)));
title('Spikes');
subplot(1,3,3)
imshow(reconstruct_images(:,:,T),[]);
title('Reconstruction');
axis equal;
axis([0 myVid.Width 0 myVid.Height])

