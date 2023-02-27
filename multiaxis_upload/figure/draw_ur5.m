for i= 1:size(motion,1)
    tcp_real = motion(i,4:6);
    tcp_pred = pred(i,5:7);

    rot_real = rovec2mat(tcp_real);
    rot_pred = rovec2mat(tcp_pred);

    r = rot_real(:,3);
    r1 = r(1);
    r2 = r(2);
    r3 = r(3);

    p = rot_pred(:,3);
    p1 = p(1);
    p2 = p(2);
    p3 = p(3);

    if any(isnan(r)) == 0 && any(isnan(p)) == 0
        z_real(i,:) = r;
        z_pred(i,:) = p;
        angle_real(i) = acos(r' * [0 0 1]') / pi * 180;
        angle_pred(i) = acos(p' * [0 0 1]') / pi * 180;
        z_err(i) = acos(r' * p);
    else
        angle_real(i) = 0;
        angle_pred(i) = 0;
        z_err(i) = -10;
    end

end

% force = sensor(:,17);
% forcex = force(force>2.9);
% zr = z_err(force>2.9);
% zc = z_err / pi * 180;
% zc(force<2.9) = 0;
% colormap("parula")
% plot(tim2, zc, 'LineWidth', 2)
% ylabel('Error Angle (deg)')
% xlabel('Time (sec)')
% set(gca, 'box', 'off', 'FontSize', 25)
% 
% x0=1;
% y0=10;
% width=2000;
% height=400;
% set(gcf,'position',[x0,y0,width,height])

plot(tim2, angle_real, 'LineWidth', 2)
hold on
plot(tim2, angle_pred, 'LineWidth', 2)
ylabel('Angle (deg)')
xlabel('Time (sec)')
legend('Real Angle', 'Predicted Angle')
set(gca, 'box', 'off', 'FontSize', 25)
