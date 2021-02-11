
tMax = 1;
dt = 0.01;
T = dt:dt:tMax;

S1 = 2*sin(2*pi*T) - 0.5*cos(6*pi*T) + 0.75*sin(10*pi*T);
S2 = -1*sin(2*pi*T) + 0.75*sin(2*pi*T) - 0.05*rand(1,length(T));
S3 = normpdf(T,0.2,0.1) + 0.5*normpdf(T,0.75,0.1) - 0.1*rand(1,length(T));

factor = 1.05;
[spikes_TBR_1, threshold] = TemporalContrastEncode(S1,factor);
signal_TBR_1 = TemporalContrastDecode(spikes_TBR_1, threshold);
factor = 1;
[spikes_TBR_2, threshold] = TemporalContrastEncode(S2,factor);
signal_TBR_2 = TemporalContrastDecode(spikes_TBR_2, threshold);
factor = 0.95;
[spikes_TBR_3, threshold] = TemporalContrastEncode(S3,factor);
signal_TBR_3 = TemporalContrastDecode(spikes_TBR_3, threshold);

subplot(9,3,[1,4])
hold on;
plot(T,S1)
plot(T,signal_TBR_1)
hold off;
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

