w = logspace(-1,1,12052);
[mag,phase,wout] = bode(tf8, w);                     % Get Plot Data
mag = squeeze(mag);                                             % Reduce (1x1xN) Matrix To (1xN)
phase= squeeze(phase);
magr2 = (mag/max(mag)).^2;                                      % Calculate Power Of Ratio Of ‘mag/max(mag)’
dB3 = interp1(magr2, [wout phase mag], 0.5, 'spline');          % Find Frequency & Phase & Amplitude of Half-Power (-3 dB) Point

figure(1)
colormap("parula")
subplot(2,1,1)

loglog(wout/10, mag, '-b',  'LineWidth', 2)
hold on
a = loglog([dB3(1)/10 dB3(1)/10], [0.000001 2], 'LineStyle','--');

ylim([0.09 1.5])
ylabel('Amplitude')
set(gca, 'box', 'off', 'FontSize', 25)

subplot(2,1,2)
semilogx(wout/10, phase, '-b',  'LineWidth', 2)
hold on
b = loglog([dB3(1)/10 dB3(1)/10], [10 -100], 'LineStyle','--');
legend([a(1);b(1)], ' Cutoff Frequency')
ylim([-100 10])
ylabel('Phase (deg)')
xlabel('Frequency (Hz)')

set(gca, 'box', 'off', 'FontSize', 25)