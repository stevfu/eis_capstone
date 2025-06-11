clear all; clc;

 
Rs = 13888.889;   % electrolyte resistance (Ohms)
Rct = 12800; % Charge transfer resistance (Ohms)
Cdl = 70.8e-6; % Double layer capacitance (Farads)


s = tf('s');

numZ = [  Rs*Rct*Cdl,   Rs + Rct ];

denZ = [     Rct*Cdl,       1    ];

Z_tf = tf(numZ, denZ);


disp('The transfer function (Impedance Z(s)) is:');
Z_tf


f_start = .1;   % Start frequency sweep in Hz 
f_end = 10000; % End frequency sweep in Hz 
n_points = 100;   % Number of frequency points, N

%frequency vector
freq_hz = logspace(log10(f_start), log10(f_end), n_points);
omega = 2 * pi * freq_hz; % Convert to angular frequency 

% freqresp calculates the response H(jw) at specified frequencies
% The output 'resp' is complex 
resp = freqresp(Z_tf, omega);

Z_complex = squeeze(resp);

% Extract Real and Imaginary parts
Z_real = real(Z_complex);
Z_imag = imag(Z_complex); 



wRealWeight  = 1;
wImagWeight  = 1;
dataFile = 'Overnight.csv';
d1 = 'testdata.csv';
d2 = '5HourTest_n 1.csv';

%selectedModel = 'Gerischer'; 

D      = readmatrix(dataFile);
D_1    = readmatrix(d1);
D_2    = readmatrix(d2);

freq  = D(:,2);
Zr    = D(:,6);
Zi    = -D(:,7);                             % Invert sign if CSV stored +Im(Z)

freq1  = D_1(:,2);
Zr1    = D_1(:,6);
Zi1    = -D_1(:,7);  

freq2  = D_2(:,2);
Zr2    = D_2(:,6);
Zi2    = -D_2(:,7);  

[freq, sortIdx] = sort(freq);                % Ensure frequency is sorted
Zr = Zr(sortIdx);
Zi = Zi(sortIdx);

[freq1, sortIdx1] = sort(freq1);                % Ensure frequency is sorted
Zr1 = Zr1(sortIdx1);
Zi1 = Zi1(sortIdx1);


[freq2, sortIdx2] = sort(freq2);                % Ensure frequency is sorted
Zr2 = Zr2(sortIdx2);
Zi2 = Zi2(sortIdx2);


windowSize =1001;
Zr_smooth = movmean(Zr, windowSize);
Zi_smooth = movmean(Zi, windowSize);

Zr_smooth1 = movmean(Zr1, windowSize);
Zi_smooth1 = movmean(Zi1, windowSize);

Zr_smooth2 = movmean(Zr2, windowSize);
Zi_smooth2 = movmean(Zi2, windowSize);
Zexp = Zr_smooth + 1j * Zi_smooth;

figure('Name', 'Nyquist ');
plot(Zr_smooth, Zi_smooth, 'o', 'DisplayName','Data'); hold on;
plot(Zr_smooth, Zi_smooth, '-', 'LineWidth',1.5, 'DisplayName','Model Fit');
plot(Zr_smooth1, Zi_smooth1, 'o', 'DisplayName','Data'); hold on;
plot(Zr_smooth1, Zi_smooth1, '-', 'LineWidth',1.5, 'DisplayName','Model Fit');
plot(Zr_smooth2, Zi_smooth2, 'o', 'DisplayName','Data'); hold on;
plot(Zr_smooth2, Zi_smooth2, '-', 'LineWidth',1.5, 'DisplayName','Model Fit');

plot(Z_real, -Z_imag, '-o'); % Plot Z_real vs -Z_imag

plot(Z_real(1), -Z_imag(1), 'sg', 'MarkerFaceColor', 'g', 'MarkerSize', 8); 
plot(Z_real(end), -Z_imag(end), 'sr', 'MarkerFaceColor', 'r', 'MarkerSize', 8); 

axis equal; grid on;
xlabel('Z_{real} [\Omega]'); ylabel('-Z_{imag} [\Omega]');
title('Nyquist Plot'); legend('Location','best');

