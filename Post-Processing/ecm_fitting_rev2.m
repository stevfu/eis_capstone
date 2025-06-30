 clear all; clc;
 
%Validate and Prove
Rs = 1000; % electrolyte resistance (Ohms)
Rct = 1e4; % Charge transfer resistance (Ohms)
Cdl = 15e-6; % Double layer capacitance (Farads)


s = tf('s');

%Validate
numZ = [Rs*Rct*Cdl,Rs + Rct];
denZ = [Rct*Cdl,1];
Z_tf = tf(numZ, denZ);


disp('The transfer function (Impedance Z(s)) is:');
Z_tf

%setup
f_start = .00001;   % Start frequency sweep in Hz 
f_end = 100000; % End frequency sweep in Hz 
n_points = 680;   % Number of frequency points, N

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
dataFile = '5HourTest_n.csv';

D = readmatrix(dataFile);

freq  = D(:,2);
Zr    = abs(D(:,6));
Zi    = abs(D(:,7));                             % Invert sign if CSV stored +Im(Z)

[freq, sortIdx] = sort(freq);                % Ensure frequency is sorted
Zr = Zr(sortIdx);
Zi = Zi(sortIdx);


windowSize =7;
Zr_smooth = movmean(Zr, windowSize);
Zi_smooth = movmean(Zi, windowSize);

%%Kramer kronig
%Zre = (1/pi)*

Zexp = Zr_smooth + 1j * Zi_smooth;

figure('Name', 'Nyquist ');
%plot(Zr_smooth, Zi_smooth, 'o', 'DisplayName','Data'); hold on;
%plot(Zr_smooth, Zi_smooth, '-', 'LineWidth',1.5, 'DisplayName','Model Fit');


% plot(Z_real, -Z_imag, '-o'); % Plot Z_real vs -Z_imag
% hold on;

plot(Zr_smooth, Zi_smooth, '-o'); 
axis equal; grid on;
xlabel('Z_{real} [\Omega]'); ylabel('-Z_{imag} [\Omega]');
title('Nyquist Plot'); legend('Location','best');


hold off;

realZ = Zr_smooth;
imagZ = Zi_smooth;
% realZ = Zr;
% imagZ = Zi;
freqExt = freq;
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Name','Impedance Spectroscopy','NumberTitle','on');
set(gca,'FontSize',12,'LineWidth',2,'Color',[1 1 1],'Box','on');
h = semilogx(freqExt,realZ);
set(h,'LineWidth',4,'LineStyle','-','Color','b')
hold on;
h = semilogx(freqExt,imagZ);
set(h,'LineWidth',4,'LineStyle','--','Color','r')
grid on
title('Impedance Spectroscopy','fontsize',12,'fontweight','n','color','k');
xlabel('Frequency  [Hz]','fontsize',12,'fontweight','n','color','k');
ylabel('Impedance  [\Omega]','fontsize',12,'fontweight','n','fontangle','n','color','k');

h = legend('Real Z','Imag Z');
set(h,'Box','on','Color','w','Location','NorthEast','FontSize',12,'FontWeight','n','FontAngle','n')

xlim([0.01 100e3])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Removing the series resistor (It improves the precision in high frequencies)
realZ_RemovedSeries = realZ; %- realZ(end);

%%% Calculating the imaginary part using the KK relations. 
NumFreq = length(freqExt);
FreqKK = freq(1+2:NumFreq-2);
KKimagZ = zeros(1,length(FreqKK));
for nn = 3:NumFreq-2
    integrand = realZ_RemovedSeries./(freqExt.^2 - freqExt(nn)^2);
    KKimagZ(nn - 2) = (2*freqExt(nn)/pi)*(trapz(freqExt(1:nn-1),integrand(1:nn-1)) + trapz(freqExt(nn+1:NumFreq),integrand(nn+1:NumFreq)));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Name','Impedance Spectroscopy','NumberTitle','on');
set(gca,'FontSize',12,'LineWidth',2,'Color',[1 1 1],'Box','on');
h = semilogx(freq,imagZ);
set(h,'LineWidth',2.5,'LineStyle','--','Color','r')
hold on;
h = semilogx(FreqKK,KKimagZ);
set(h,'LineWidth',2.5,'LineStyle','-','Color','g')
grid on;

title('Impedance Spectroscopy','fontsize',12,'fontweight','n','color','k');
xlabel('Frequency  [Hz]','fontsize',12,'fontweight','n','color','k');
ylabel('Reactance  [\Omega]','fontsize',12,'fontweight','n','fontangle','n','color','k');

h = legend('Measurement','Kramers-Kronig');
set(h,'Box','on','Color','w','Location','NorthEast','FontSize',12,'FontWeight','n','FontAngle','n')

xlim([1 max(freq)])

%%% Calculating the real part using the KK relations.  
imagZ_RemovedSeries = imagZ;
NumFreq = length(freqExt);
FreqKK = freq(1+2:NumFreq-2);
KKrealZ = zeros(1,length(FreqKK));
for nn = 3:NumFreq-2
    integrand = imagZ_RemovedSeries.*freqExt./(freqExt.^2 - freqExt(nn)^2);
    KKrealZ(nn - 2) = -(2/pi)*(trapz(freqExt(1:nn-1),integrand(1:nn-1)) + trapz(freqExt(nn+1:NumFreq),integrand(nn+1:NumFreq)));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Name','Impedance Spectroscopy','NumberTitle','on');

set(gca,'FontSize',12,'LineWidth',2,'Color',[1 1 1],'Box','on');

h = semilogx(freq,realZ);
set(h,'LineWidth',2.5,'LineStyle','--','Color','r')
hold on;
h = semilogx(FreqKK,KKrealZ + realZ(end));
set(h,'LineWidth',2.5,'LineStyle','-','Color','g')
grid on;

title('Impedance Spectroscopy','fontsize',12,'fontweight','n','color','k');
xlabel('Frequency  [Hz]','fontsize',12,'fontweight','n','color','k');
ylabel('Resistance  [\Omega]','fontsize',12,'fontweight','n','fontangle','n','color','k');

h = legend('Measurement','Kramers-Kronig');
set(h,'Box','on','Color','w','Location','NorthEast','FontSize',12,'FontWeight','n','FontAngle','n')

hold on;

% Calculate residuals
% Align measured and KK data: use indices 3:end-2 for measured data
res_real_pct = 100 * (realZ(3:end-2) - (KKrealZ(:) + realZ(end))) ./ abs(realZ(3:end-2));
res_imag_pct = 100 * (imagZ(3:end-2) - KKimagZ(:)) ./ abs(imagZ(3:end-2));


figure('Name','Percentage Residuals','NumberTitle','on');
subplot(2,1,1);
semilogx(FreqKK, res_real_pct, 'b', 'LineWidth', 2.5);
grid on;
title('Real Part Residuals (%)');
xlabel('Frequency [Hz]');
ylabel('Residual (%)');

subplot(2,1,2);
semilogx(FreqKK, res_imag_pct, 'r', 'LineWidth', 2.5);
grid on;
title('Imaginary Part Residuals (%)');
xlabel('Frequency [Hz]');
ylabel('Residual (%)');

% Excellent/Reference	< 1%	Ideal, rarely achieved
% Good/Valid	< 3%	Acceptable for most studies
% Marginal	3–10%	Caution, check for issues
% Poor/Invalid	> 10%	Likely invalid data

% Fit 2-RC ECM model
[params, rmse, Z_fit] = eis_ecm_fit(freq, Zr, Zi, 2);

% Compare with analytical solution
residuals = [Zr - real(Z_fit); Zi - imag(Z_fit)];


function [params, rmse, Z_fit] = eis_ecm_fit(frequency, Z_real, Z_imag, model_order)
% EIS_ECM_FIT - Fits Equivalent Circuit Model parameters to EIS data
%
% Inputs:
%   frequency   : Measured frequency vector (Hz)
%   Z_real      : Real part of impedance (Ω)
%   Z_imag      : Imaginary part of impedance (Ω)
%   model_order : Number of RC pairs in model (1, 2, or 3)

% Outputs:
%   params      : Fitted ECM parameters [R0, R1, C1, ..., Rn, Cn]
%   rmse        : Root Mean Square Error of fit
%   Z_fit       : Fitted impedance values

% Sort data by descending frequency
[f, idx] = sort(frequency, 'descend');
Z = Z_real(idx) + 1i*Z_imag(idx);

% Angular frequency
w = 2*pi*f;

% Set up ECM model based on selected order
switch model_order
    case 1
        model = @(x,w) x(1) + x(2)./(1 + 1i*w*x(2)*x(3));
        lb = [0, 0, 0];
        ub = [Inf, Inf, Inf];
        x0 = [min(Z_real), (max(Z_real)-min(Z_real))/2, 1e-3];
    case 2
        model = @(x,w) x(1) + x(2)./(1 + 1i*w*x(2)*x(3)) + ...
                         x(4)./(1 + 1i*w*x(4)*x(5));
        lb = [0, 0, 0, 0, 0];
        ub = [Inf, Inf, Inf, Inf, Inf];
        % Initial parameter estimates
        R0_est = min(Z_real);
        R1_est = (max(Z_real) - R0_est) * 0.4;
        R2_est = (max(Z_real) - R0_est) * 0.6;
        x0 = [R0_est, R1_est, 1e-3, R2_est, 1e-1];
end

% Multi-start optimization to avoid local minima
options = optimoptions('lsqcurvefit', 'Algorithm', 'levenberg-marquardt', ...
                      'Display', 'off', 'MaxIterations', 1000);
problem = createOptimProblem('lsqcurvefit', 'objective', @(x,w) [real(model(x,w)); imag(model(x,w))], ...
                            'x0', x0, 'lb', lb, 'ub', ub, ...
                            'xdata', w, 'ydata', [real(Z); imag(Z)]);

% Create hybrid optimization with patternsearch
ms = MultiStart('UseParallel', true, 'Display', 'off');
[params, resnorm] = run(ms, problem, 5); % 5 starting points

% Calculate fitted impedance
Z_fit = model(params, w);
Z_fit_real = real(Z_fit);
Z_fit_imag = imag(Z_fit);

% Calculate RMSE
residuals_real = Z_real - Z_fit_real;
residuals_imag = Z_imag - Z_fit_imag;
rmse = sqrt(mean([residuals_real; residuals_imag].^2));

% Plot results
figure;
plot(Z_real, -Z_imag, 'o', 'DisplayName', 'Measured');
hold on;
plot(Z_fit_real, -Z_fit_imag, 'r-', 'DisplayName', 'Fitted');
set(gca, 'YDir', 'normal');
xlabel('Z_{real} (\Omega)');
ylabel('-Z_{imag} (\Omega)');
title('Nyquist Plot with ECM Fit');
legend('Location', 'best');
grid on;

% Display parameters
fprintf('\nFitted ECM Parameters:\n');
fprintf('R0 = %.4f Ω\n', params(1));
for i = 1:model_order
    fprintf('R%d = %.4f Ω, C%d = %.4e F\n', i, params(2*i), i, params(2*i+1));
end
fprintf('RMSE: %.4f\n', rmse);
end

