% Advanced EIS Post-Processing Script for Experimental Data
clear all; clc;

%% 1) User Settings & Metadata
Rs_expected  = 13888.889;       % Ohm
Rct_expected = 128000;     % Ohm
Cdl_expected = 70.4e-6;     % F
wRealWeight  = 1;
wImagWeight  = 1;
% dataFile = 'Balloon2.csv';
dataFile = 'QuilWater.csv';
% dataFile = 'BennyWater.csv';

selectedModel = '2RC'; % Options: '2RC', 'R-C', 'Warburg', 'Gerischer', 'DRT'

%% 2) Load & Inspect Data
D     = readmatrix(dataFile);
freq  = D(:,2);
Zr    = -D(:,6);
Zi    = -D(:,7);                             % Invert sign if CSV stored +Im(Z)

% Sort by frequency
[freq, sortIdx] = sort(freq);                % Ensure frequency is sorted
Zr = Zr(sortIdx);
Zi = Zi(sortIdx);

% Apply moving average filter
windowSize = 31;
Zr_smooth = movmean(Zr, windowSize);
Zi_smooth = movmean(Zi, windowSize);
Zexp = Zr_smooth + 1j * Zi_smooth;

N = numel(freq);


%% 3) Equivalent Circuit Model Selection
switch selectedModel
    case '2RC'
        modelFun = @(p,f) p(1) + 1./(1/p(2) + 1j*2*pi*f*p(3)) + 1./(1/p(4) + 1j*2*pi*f*p(5));
        p0 = [Rs_expected, Rct_expected, Cdl_expected, Rct_expected, Cdl_expected];
        lb = [0, 0, Cdl_expected*1e-3, 0, Cdl_expected*1e-3];
        ub = [10*Rs_expected, 10*Rct_expected, Cdl_expected*100, 10*Rct_expected, Cdl_expected*100];
        fNames = {'Rs','Rct1','Cdl1','Rct2','Cdl2'};
        units  = {'Ω','Ω','F','Ω','F'};

    case 'R-C'
        modelFun = @(p,f) p(1) + 1./(1/p(2) + 1j*2*pi*f*p(3));
        p0 = [Rs_expected, Rct_expected, Cdl_expected];
        lb = [0, 0, Cdl_expected*1e-3];
        ub = [10*Rs_expected, 10*Rct_expected, Cdl_expected*100];
        fNames = {'Rs','Rct','Cdl'};
        units  = {'Ω','Ω','F'};

    case 'Warburg'
        modelFun = @(p,f) p(1) + 1./(1/p(2) + 1j*2*pi*f*p(3)) + p(4)./(sqrt(1j*2*pi*f));
        p0 = [Rs_expected, Rct_expected, Cdl_expected, 1];
        lb = [0, 0, Cdl_expected*1e-3, 0];
        ub = [10*Rs_expected, 10*Rct_expected, Cdl_expected*100, 1e3];
        fNames = {'Rs','Rct','Cdl','W'};
        units  = {'Ω','Ω','F','Ω·s^{-1/2}'};

    case 'Gerischer'
        modelFun = @(p,f) p(1) + 1./(1/p(2) + 1j*2*pi*f*p(3)) + p(4)./sqrt(1 + 1j*2*pi*f*p(5));
        p0 = [Rs_expected, Rct_expected, Cdl_expected, 1, 1e-3];
        lb = [0, 0, Cdl_expected*1e-3, 0, 1e-6];
        ub = [10*Rs_expected, 10*Rct_expected, Cdl_expected*100, 1e3, 1];
        fNames = {'Rs','Rct','Cdl','G','tau'};
        units  = {'Ω','Ω','F','Ω','s'};

    case 'DRT'
        % DRT grid setup
        nDRT = 40; % number of tau points
        tau_grid = logspace(-6, 2, nDRT); % log-spaced tau (1 µs to 100 s)
        
        % Model function: Rs + sum_i (gamma_i / (1 + j2πfτ_i))
        modelFun = @(p,f) p(1) + sum(p(2:end) .* (1 ./ (1 + 1j*2*pi*f(:) * tau_grid)), 2);

        % Initial guesses and bounds
        p0 = [Rs_expected, ones(1, nDRT)*1]; % Rs + gamma
        lb = [0, zeros(1,nDRT)];
        ub = [10*Rs_expected, ones(1,nDRT)*1e3]; % Adjust max gamma if needed
        
        % Parameter names for display
        fNames = ['Rs', strcat("γ_", string(1:nDRT))];
        units = [{'Ω'}, repmat({'Ω'}, 1, nDRT)];

    otherwise
        error('Unknown model selection');
end

%% 4) Least-Squares Optimization
objFun = @(p) [wRealWeight * (real(modelFun(p,freq)) - real(Zexp));
               wImagWeight * (imag(modelFun(p,freq)) - imag(Zexp))];
opts = optimoptions('lsqnonlin', 'Display', 'off', 'MaxFunctionEvaluations', 1e4, 'FunctionTolerance', 1e-12);
[p_fit, ~, resnorm, residuals, ~, ~, jacobian] = lsqnonlin(objFun, p0, lb, ub, opts);

%% 5) Fit Statistics
SSR      = sum(residuals.^2);
dof      = 2*N - numel(p_fit);
chi2_red = SSR / dof;
covParam = inv(jacobian' * jacobian) * (SSR / dof);
paramSE  = sqrt(diag(covParam));

Zmag_exp = abs(Zexp); Zmag_fit = abs(modelFun(p_fit, freq));
RMSE_mag = sqrt(mean((Zmag_exp - Zmag_fit).^2));
R2_mag = 1 - sum((Zmag_exp - Zmag_fit).^2) / sum((Zmag_exp - mean(Zmag_exp)).^2);

%% 6) Display Parameters & Fit Quality
fprintf('\nFitted Parameters with 1σ uncertainties:\n');
p_fit = full(p_fit);
paramSE = full(paramSE);
for k = 1:numel(p_fit)
    fprintf('%4s = %.6g %s (± %.2g)\n', fNames{k}, p_fit(k), units{k}, paramSE(k));
end

fprintf('\nFit Statistics:\n');
fprintf('Residual sum of squares = %.4g\n', SSR);
fprintf('Reduced chi-squared = %.4g\n', chi2_red);
fprintf('RMSE (|Z|) = %.4g\n', RMSE_mag);
fprintf('R² (|Z|) = %.4f\n', R2_mag);



%% 8) Nyquist and Bode Plots
Zfit = modelFun(p_fit, freq);
figure('Name', 'Nyquist');
plot(real(Zexp), -imag(Zexp), 'o', 'DisplayName','Data'); hold on;
plot(real(Zfit), -imag(Zfit), '-', 'LineWidth',1.5, 'DisplayName','Model Fit');
axis equal; grid on;
xlabel('Z_{real} [\Omega]'); ylabel('-Z_{imag} [\Omega]');
title('Nyquist Plot'); legend('Location','best');

figure('Name', 'Bode');
subplot(2,1,1);
loglog(freq, abs(Zexp), 'o', freq, abs(Zfit), '-', 'LineWidth',1.5);
ylabel('|Z| [\Omega]'); title('Bode - Magnitude'); grid on; legend('Data','Fit');
subplot(2,1,2);
semilogx(freq, angle(Zexp)*180/pi, 'o', freq, angle(Zfit)*180/pi, '-', 'LineWidth',1.5);
xlabel('Frequency [Hz]'); ylabel('Phase [°]'); title('Bode - Phase'); grid on; legend('Data','Fit');


% %% 9) Robust Loewner Framework Approximation
% 
% % Define s-domain
% s = 1j * 2 * pi * freq;
% N = numel(s);
% 
% % Split data into left and right sets
% left_idx = 1:2:N;
% right_idx = 2:2:N;
% if mod(N,2) == 1
%     left_idx(end) = []; % Ensure even pairing
% end
% 
% sL = s(left_idx);  ZL = Zexp(left_idx);
% sR = s(right_idx); ZR = Zexp(right_idx);
% 
% % Construct Loewner and shifted Loewner matrices
% [SL, SR] = meshgrid(sL, sR);
% [ZL_grid, ZR_grid] = meshgrid(ZL, ZR);
% 
% L  = (ZL_grid - ZR_grid.') ./ (SL - SR.');  % Loewner matrix
% Ls = (SL .* ZL_grid - SR .* ZR_grid.') ./ (SL - SR.');
% % Perform SVD and rank truncation
% [U, S, V] = svd(L, 'econ');
% sing_vals = diag(S);
% 
% tol = 1e-10 * max(sing_vals);  % <- less aggressive
% r = sum(sing_vals > tol);
% 
% fprintf('Loewner reduced rank: r = %d\n', r);  % Debug
% 
% Ur = U(:, 1:r); Vr = V(:, 1:r);
% Sr = S(1:r, 1:r);
% Er = Ur' * L  * Vr;
% Ar = Ur' * Ls * Vr;
% 
% Z_loewner = arrayfun(@(sk) ones(1, r) * ((sk * Er - Ar) \ ones(r, 1)), s);
% 
% [Vecs, Vals] = eig(Ar, Er);
% lambda = diag(Vals);              
% weights = Vecs \ ones(r, 1);      
% 
% tau_eig = 1 ./ abs(lambda);       
% gamma_eig = abs(weights);         
% 
% % Robust eigenvalue filtering
% valid_idx = (imag(tau_eig) < 1e-3) & (real(tau_eig) > 0);
% tau_eig = real(tau_eig(valid_idx));
% gamma_eig = real(gamma_eig(valid_idx));
% 
% [tau_eig, sortIdx] = sort(tau_eig);
% gamma_eig = gamma_eig(sortIdx);
% 
% %% --- PLOTS ---
% figure('Name', 'DRT via Loewner Eigen-Decomposition');
% semilogx(tau_eig, gamma_eig, 's-', 'LineWidth', 1.5);
% xlabel('\tau [s]'); ylabel('\gamma_{eig}(\tau) [\Omega]');
% title('DRT via Loewner Eigen-Decomposition'); grid on;
% 
% figure('Name', 'Nyquist with Loewner');
% plot(real(Zexp), -imag(Zexp), 'o', 'DisplayName','Data'); hold on;
% plot(real(Zfit), -imag(Zfit), '-', 'LineWidth',1.5, 'DisplayName','Model Fit');
% plot(real(Z_loewner), -imag(Z_loewner), '--', 'LineWidth',1.5, 'DisplayName','Loewner Model');
% axis equal; grid on;
% xlabel('Z_{real} [\Omega]'); ylabel('-Z_{imag} [\Omega]');
% title('Nyquist Plot: Model Fit vs. Loewner'); legend('Location','best');

% %% analytical model
% 
% Rs = 923626;   % electrolyte resistance (Ohms)
% Rct = 1280000; % Charge transfer resistance (Ohms)
% Cdl = 70.8e-6; % Double layer capacitance (Farads)
% 
% 
% s = tf('s');
% 
% numZ = [Rs*Rct*Cdl, Rs + Rct];
% 
% denZ = [Rct*Cdl,1];
% 
% Z_tf = tf(numZ, denZ);
% 
% 
% disp('The transfer function (Impedance Z(s)) is:');
% Z_tf
% 
% 
% f_start = 0.00001;   % Start frequency sweep in Hz 
% f_end = 100000; % End frequency sweep in Hz 
% n_points = 100;   % Number of frequency points, N
% 
% %frequency vector
% freq_hz = logspace(log10(f_start), log10(f_end), n_points);
% omega = 2 * pi * freq_hz; % Convert to angular frequency 
% 
% % freqresp calculates the response H(jw) at specified frequencies
% % The output 'resp' is complex 
% resp = freqresp(Z_tf, omega);
% 
% Z_complex = squeeze(resp);
% 
% % Extract Real and Imaginary parts
% Z_real = real(Z_complex);
% Z_imag = imag(Z_complex); 
% 
% % Generate the Nyquist Plot 
% % Method 1: Using the built-in 'nyquist' function for quick plotting
% figure;
% nyquistplot(Z_tf, {2*pi*f_start, 2*pi*f_end}); % Provide frequency range in rad/s
% title('Nyquist Plot (using nyquistplot function)');
% xlabel('Real(Z) / Ohms');
% ylabel('Imaginary(Z) / Ohms'); % nyquistplot by default plots Im(Z)
% grid on;
% axis equal; 
% 
% % Method 2: Plotting manually for more control (e.g., plotting -Z_imag)
% figure;
% plot(Z_real, -Z_imag, '-o'); % Plot Z_real vs -Z_imag
% hold on;
% 
% plot(Z_real(1), -Z_imag(1), 'sg', 'MarkerFaceColor', 'g', 'MarkerSize', 8); 
% plot(Z_real(end), -Z_imag(end), 'sr', 'MarkerFaceColor', 'r', 'MarkerSize', 8); 
% hold off;
% 
% title('Nyquist Plot (manual: Z_{real} vs -Z_{imag})');
% xlabel('Z_{real} / Ohms');
% ylabel('-Z_{imag} / Ohms');
% grid on;
% axis equal; 
% legend('Impedance Data', 'Start Frequency', 'End Frequency', 'Location', 'best');
% 
% disp('Frequency (Hz), Real(Z) (Ohms), Imaginary(Z) (Ohms):');
% for i = 1:length(freq_hz)
%     fprintf('%10.4f Hz, %10.4f Ohms, %10.4f Ohms\n', freq_hz(i), Z_real(i), Z_imag(i));
% end

%% 10) Export Summary Table
if numel(p_fit) == numel(paramSE) && numel(p_fit) == numel(fNames)
    T = table(fNames', full(p_fit(:)), full(paramSE(:)), 'VariableNames', {'Parameter','Estimate','StdError'});
    writetable(T, 'eis_fitted_parameters.csv');
else
    warning('Mismatch in fitted parameter lengths. Table not saved.');
end
