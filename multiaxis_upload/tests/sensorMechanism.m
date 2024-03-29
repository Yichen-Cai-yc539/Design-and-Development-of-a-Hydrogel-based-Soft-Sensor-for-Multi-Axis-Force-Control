clear

load('tcp_net.mat')

dq = daq("ni");
rate = 2.5*10000;
dq.Rate = rate;
for i = 0:1:15
    chn = sprintf('ai%d', i);
    chi = addinput(dq, 'Dev5', chn, 'Voltage');
    chi.TerminalConfig = 'SingleEnded';
end

% GND force strain gauge initialization
for i = 0:1:3
    chn = sprintf('ai%d', i);
    chi = addinput(dq, 'Dev4', chn, 'Voltage');
    chi.TerminalConfig = "SingleEnded";
end

% MoveRobot_KI position initialization
ready_tran = [89.11 449.81 92];
ready_rota = [0 0 0];

% MoveRobot_ORI initialization
pos = [0 0 0];
bounds = [-0.3 -0.3  0  80;
    0.3  0.3  0  86];

% Start Robot
Robot_IP = '169.254.187.130';
Socket_conn = tcpip(Robot_IP,30010,'NetworkRole','server');
fclose(Socket_conn);
disp('Press Play on Robot...')
fopen(Socket_conn);
disp('Connected! Start random walk on UR5');
rota = get_pos(Socket_conn); % Read junk

% Go to ready tran & rota
for i = 1:100
    moverobot_Ki(Socket_conn, ready_tran, ready_rota)
end

net = resetState(net);

% for i=1:10
%     x{i}=x{1, 1}(:,1);
% end

%des=1.60;

tic

tim1(1)=0;
tim2(1)=0;


[sensor(1, :)] = read(dq, "OutputFormat", "Matrix");

for i=1:1

    walk = [0 0.25 0;
            0.25 0 0;
            0 -0.25 0;
            -0.25 0 0];
    tx([1:size(walk,1)],1) = ready_tran(1);
    ty([1:size(walk,1)],1) = ready_tran(2);
    tz = [92 92 92 92]';
    Tran_array = [ready_tran; [tx ty tz]];
    walk = [[0 0 0]; walk];

    flag = 2;

    tic

    for t = 2:size(Tran_array, 1)
        message = sprintf('Moving to press no. %d', t-1);
        disp(message);
        n = randi(50);
        m = randi(10);
        total = 150 + randi(50)+30;
        get = 0;
        for j = 1:total
            if j < 60 % Initialize tran & rota
                trans = ready_tran;
                rota = ready_rota;
                mode = 2;
            elseif j == 60 % Rotate to new orientation
                wk = walk(t,:);
                mode = 1;
            elseif j < 70+n % Moves down j = 70 - 120; duration 10 - 60
                if get == 0
                    emp = [];
                    while length(emp) < 6
                        emp = get_pos(Socket_conn);
                    end
                end
                rota = emp(4:6);
                get = 1;
                path_z = linspace(ready_tran(3), Tran_array(t,3), 10+n);
                trans = [Tran_array(t,1:2) path_z(j-60)];
                mode = 0;
            elseif j < 130+m % Stays down j = 71 - 140; duration 10 - 59
                trans = Tran_array(t,:);
                mode = 0;
            elseif j < total % Moves up j = 140 - 200; duration 10 - 60
                path_z = linspace(Tran_array(t,3), ready_tran(3), total-129-m);
                trans = [Tran_array(t,[1:2]) path_z(j-(129+m))];
                mode = 0;
            elseif j == total
                wk = invert(walk(t,:));
                mode = 1;
            end
            if mode == 0
                moverobot_Ki(Socket_conn, trans, rota);
            elseif mode == 1

                moverobot_ori(Socket_conn, pos, wk);
                pause(10)

            end

            % Read data
            [sensor(flag, :,i)] = read(dq, "OutputFormat", "Matrix");
            [motion(flag, :,i)] = [trans rota];

            tim2(flag,i)=toc;
            %             data2(flag,:,i)=sensor(flag, 1:16, i);
            data2(flag,:,i)=sensor(flag-1,1:16,i)+(sensor(flag,1:16,i)-sensor(flag-1,1:16,i))*(1/20)/(tim2(flag,i)-tim2(flag-1,i));%freq=25
            %   x{1}=(data2(flag,:)'-mu)./sig;
            rtim_sig{i}(flag,:) = std(data2(:,:,i)',0,2);
            x{i}=(data2(flag,:,i)'-sensor(2, 1:16,i)')./sig;%



            % [YPred_o]= predict(net,x,'MiniBatchSize',10);
            [net,YPred_o]= predictAndUpdateState(net,x{i},'MiniBatchSize',1);



            pred(flag,:,i)=YPred_o.*sig2+mu2;

            %

            % Compare pred_cart with real_cart
%             m1 = motion(flag,4,i);
%             m2 = motion(flag,5,i);
%             m3 = motion(flag,6,i);
%             [cx,cy,cz] = sph2cart(m1, m2, m3);
%             real_cart(flag,:,i) = [cx,cy,cz];
            %
            %tim(flag, k) = toc;

            flag = flag + 1;
        end
    end
%     save realtime_test_batch.mat
%     rp = randi(120);
%     p = rp + 30;
%     pause(p)


    %     err=pred(i,1)-des(i);
    %     K=round(err*25);
    %     if err>0.01
    %         Trans(i+1,:)=Trans(i,:)-K*[0,0,0.1];
    %     elseif  err<0.01
    %         Trans(i+1,:)=Trans(i,:)-K*[0,0,0.1];
    %     else
    %         Trans(i+1,:)=Trans(i,:);
    %     end
    %
    %     if Trans(i+1,3)>bounds(1,3)||Trans(i+1,3)<bounds(2,3)%%%%IMP
    %         Trans(i+1,:)=Trans(i,:);
    %
    %     end



    %     if rem(floor(i/50),2)==0
    %        des(i+1)=1.6;%des(i)-0.003;
    %     else
    %        des(i+1)=1.1;%des(i)+0.003;
    %     end
    %des(i+1) = des(i);%1.3-0.2*sin(i/50);

    %des(i+1)=1.55-0.4*abs(sin(i/50));
    % des(i+1)=1.41+0.2*(sin(i/(400-(380/4000)*(i-1))));

    % des(i+1)=1.61-0.25*abs(sin(i/100))-0.25*abs(sin(i/70));
    % des(i+1)=1.61-0.4*floor(abs(2*sin(i/100))-0.00001);
    %des(i+1)=1.25;




    %freq=freq-22/20000;
    %i
end



time=toc;
h=figure;
% Plot
for i = 1:1
    
    for flag = 1:size(pred(:,:,i),1)

%         force(flag,:,i) = pred(flag,1:4,i);
%         o1 = pred(flag,5,i); %cart
%         o2 = pred(flag,6,i);
%         o3 = pred(flag,7,i);
%         [a b c] = cart2sph(o1, o2, o3);
%         tcp_orien(flag,:,i) = [a b c];
% 
%         % Calculate Direction of Force
%         rota_p = tcp_orien(flag,:,i);
%         rota = ready_rota;
%         Fo = [0 0 sum(force(flag,:,i))];
%         F_pred(flag,:,i) = f2delt(rota_p, rota, Fo);
% 
%         rota_r = motion(flag,4:6,i);
%         Fr = [0 0 sum(sensor(flag,17:20,i))];
%         F_real(flag,:,i) = f2delt(rota_r, rota, Fr);

        
        F_pred = sum(pred(flag,1:4,i))*pred(flag,5:7,i);
        px = linspace(0, F_pred(:,1));
        py = linspace(0, F_pred(:,2));
        pz = linspace(0, F_pred(:,3));

        F_real = sum(sensor(flag,17:20))*motion(flag,4:6,i);
        rx = linspace(0, F_real(:,1));
        ry = linspace(0, F_real(:,2));
        rz = linspace(0, F_real(:,3));
        
        axis equal
       xlim([-5 5])
         ylim([-5 5])
           zlim([-5 5])
%         plot3(px, py, pz, rx, ry, rz);
      h1=  quiver3(0,0,0,F_pred(:,1),F_pred(:,2),F_pred(:,3));
        hold on
         h2=       quiver3(0,0,0,F_real(:,1),F_real(:,2),F_real(:,3));
        pause(0.05);
%         drawnow
        delete(h2)
        delete(h1)
    end
end

%moverobot(Socket_conn,Translation,Orientation);
% moverobot_fast(Socket_conn,Translation,Orientation);
