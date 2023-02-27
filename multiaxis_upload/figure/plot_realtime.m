clf
clear
load 0_closedloop_pred.mat

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

real_force = real_force(10:end);
real_pred = real_pred(10:end);
real_error = real_force - real_pred;
real_mean = mean(abs(real_error));
real_std = std(real_error);
% 
% x0=1;
% y0=10;
% width=1000;
% height=1000;
% set(gcf,'position',[x0,y0,width,height])
% 
% scatter(real_pred, real_force, LineWidth=2)
% isline
% tbl = table(real_force, real_pred);
% mdl = fitlm(tbl,'linear');
% plot(mdl)
% legend('DeleteFcn)
tim2 = tim2(10:end);
colormap("parula")
yl=ylabel('Predicted Force (N)');
xl=xlabel('Real Force (N)');

set(gca, 'box', 'off', 'FontSize', 25)


plot(tim2, real_force, 'LineWidth', 2)
hold on
plot(tim2, real_pred,  'LineWidth', 2)
ylabel('Force (N)');
xlabel('Time (sec)');
legend('Real Force', 'Predicted Force')
set(gca, 'box', 'off', 'FontSize', 25)

