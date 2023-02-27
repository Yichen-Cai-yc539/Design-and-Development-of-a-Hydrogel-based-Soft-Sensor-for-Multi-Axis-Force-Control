h = figure;
for i= 1:size(motion,1)
    tcp_real = motion(i,4:6);
    tcp_pred = pred(i,5:7);
%   
    ra = tcp_real(1);
    rb = tcp_real(2);
    rc = tcp_real(3);
    [ra, rb, rc] = sph2cart(ra, rb, rc);
    tcp_real = [ra rb rc];
    
%     ra = tcp_pred(1);
%     rb = tcp_pred(2);
%     rc = tcp_pred(3);
%     [ra, rb, rc] = sph2cart(ra, rb, rc);
%     tcp_pred = [ra rb rc];

    % prediction cart2sph
%     ra = tcp_pred(1);
%     rb = tcp_pred(2);
%     rc = tcp_pred(3);
%     [ra, rb, rc] = cart2sph(ra, rb, rc);
%     tcp_pred = [ra rb rc];

    rot_real = rovec2mat(tcp_real);
    rot_pred = rovec2mat(tcp_pred);

    for j = 1:3
        r = rot_real(:,j);
        r1 = r(1);
        r2 = r(2);
        r3 = r(3);

        p = rot_pred(:,j);
        p1 = p(1);
        p2 = p(2);
        p3 = p(3);

        axis equal
        xlim([-1 1]);
        ylim([-1 1]);
        zlim([-1 1]);
        h_real(j) = quiver3(0,0,0, r1,r2,r3, 'b', 'LineWidth', 1);
        hold on
        h_pred(j) = quiver3(0,0,0, p1,p2,p3, 'r', 'LineWidth', 1);
        hold on
    end
        pause(0.001);
    for j = 1:3
        delete(h_real(j));
        delete(h_pred(j));
    end
end