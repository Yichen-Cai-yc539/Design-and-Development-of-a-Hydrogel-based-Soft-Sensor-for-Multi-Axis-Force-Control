clf
clear
load 0_step_force_test_sF04.mat
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

real_force = real_force(10:end);
real_pred = real_pred(10:end);
real_target = real_target(10:end);
tim2 = tim2(10:end);

real_error = real_force - real_pred;
mean_real = mean(abs(real_error));
std_real = std(real_error);
real_target = real_target';
desired = real_target(real_force>0.4);
real = real_force(real_force>0.4);
des_error = real - desired;
mean_des = mean(abs(des_error));
std_des = std(des_error);

for i=2:numel(real_target)-1
    if real_target(i) ~= real_target(i-1) | real_target(i) ~= real_target(i+1)
        real_tarc(i) = 0;
    else
        real_tarc(i) = real_target(i-1);
    end
end
real_tarc(numel(real_target)) = 0;

x0=1;
y0=10;
width=7000;
height=500;
set(gcf,'position',[x0,y0,width,height])



plot(tim2, real_force, LineWidth=2)
hold on
plot(tim2, real_pred, LineWidth=2)
hold on
plot(tim2, real_tarc, LineWidth=2)
colormap("parula")
lgd = legend('Ground Truth', 'Predicted Force', 'Target Force', 'Location', 'northeast');
yl=ylabel('Force (N)');
xl=xlabel('Time (sec)');

set(gca, 'box', 'off', 'FontSize', 25)