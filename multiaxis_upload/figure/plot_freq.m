clf
clear
load 0_freqswp_t800_y0-03_K04.mat

% Crude Conversion
vrange = [0.5 4.5];
frange = [0 50];
opR = 470;
opr = 100;
lbf2n = 4.448;

fspan = frange(2) - frange(1);
vspan = vrange(2) - vrange(1);
opamp = (opR + opr) / opr;

lbf = fspan / vspan / opamp; % force/volt

% force voltage init
vinit = sum([2.87 2.9275 3.119 2.93]);
vforce = sum(sensor(:,17:20), 2) - vinit;
vpred = sum(pred(:,1:4), 2) - vinit;
real_force = vforce * lbf * lbf2n;
real_pred = vpred * lbf * lbf2n;
target = target - vinit;
real_target = target * lbf * lbf2n;

real_force = real_force(45:end);
real_pred = real_pred(45:end);
real_target = real_target(45:end);
tim2 = tim2(45:end);

real_error = real_force - real_pred;
mean_real = mean(abs(real_error));
std_real = std(real_error);
des_error = real_force - real_target';
mean_des = mean(abs(des_error));
std_des = std(des_error);

x0=1;
y0=10;
width=7000;
height=500;
set(gcf,'position',[x0,y0,width,height])

plot(tim2, real_target, LineWidth=2)
hold on
plot(tim2, real_pred, LineWidth=2)
hold on
plot(tim2, real_force, LineWidth=1.6)

colormap("parula")
lgd = legend('Target Force', 'Predicted Force', 'Real Force', 'Orientation','horizontal', 'Location', 'northeast');
yl=ylabel('Force (N)');
xl=xlabel('Time (sec)');

set(gca, 'box', 'off', 'FontSize', 25)

% clf
% % Bode
% mag = log(real_force ./ real_target');
% freq = linspace(0,0.3,numel(mag));
% plot(freq(200:end-200), mag(200:end-200), 'LineWidth', 2)
% ylabel('Magnitude (dB)');
% xlabel('Frequency (Hz)');
% set(gca, 'box', 'off', 'FontSize', 25)
