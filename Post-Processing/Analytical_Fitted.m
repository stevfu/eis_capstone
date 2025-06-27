 
Rs = 923626;   % electrolyte resistance (Ohms)
Rct = 1280000; % Charge transfer resistance (Ohms)
Cdl = 70.8e-6; % Double layer capacitance (Farads)


s = tf('s');

numZ = [  Rs*Rct*Cdl,   Rs + Rct ];

denZ = [     Rct*Cdl,       1    ];

Z_tf = tf(numZ, denZ);


disp('The transfer function (Impedance Z(s)) is:');
Z_tf


f_start = .00001;   % Start frequency sweep in Hz 
f_end = 1000000; % End frequency sweep in Hz 
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

% Generate the Nyquist Plot 
% Method 1: Using the built-in 'nyquist' function for quick plotting
figure;
nyquistplot(Z_tf, {2*pi*f_start, 2*pi*f_end}); % Provide frequency range in rad/s
title('Nyquist Plot (using nyquistplot function)');
xlabel('Real(Z) / Ohms');
ylabel('Imaginary(Z) / Ohms'); % nyquistplot by default plots Im(Z)
grid on;
axis equal; 

% Method 2: Plotting manually for more control (e.g., plotting -Z_imag)
figure;
plot(Z_real, -Z_imag, '-o'); % Plot Z_real vs -Z_imag
hold on;

plot(Z_real(1), -Z_imag(1), 'sg', 'MarkerFaceColor', 'g', 'MarkerSize', 8); 
plot(Z_real(end), -Z_imag(end), 'sr', 'MarkerFaceColor', 'r', 'MarkerSize', 8); 
hold off;

title('Nyquist Plot (manual: Z_{real} vs -Z_{imag})');
xlabel('Z_{real} / Ohms');
ylabel('-Z_{imag} / Ohms');
grid on;
axis equal; 
legend('Impedance Data', 'Start Frequency', 'End Frequency', 'Location', 'best');

disp('Frequency (Hz), Real(Z) (Ohms), Imaginary(Z) (Ohms):');
for i = 1:length(freq_hz)
    fprintf('%10.4f Hz, %10.4f Ohms, %10.4f Ohms\n', freq_hz(i), Z_real(i), Z_imag(i));
end
