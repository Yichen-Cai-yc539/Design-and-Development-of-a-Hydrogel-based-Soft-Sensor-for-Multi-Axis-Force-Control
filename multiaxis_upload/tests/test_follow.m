clear
load('0_lstmnet.mat')

% Initialization
dq = daq("ni");
rate = 2.5*10000;
dq.Rate = rate;
for i = 0:1:15
    chn = sprintf('ai%d', i);
    chi = addinput(dq, 'Dev5', chn, 'Voltage');
    chi.TerminalConfig = 'SingleEnded';
end
for i = 0:1:3
    chn = sprintf('ai%d', i);
    chi = addinput(dq, 'Dev4', chn, 'Voltage');
    chi.TerminalConfig = "SingleEnded";
end

% MoveRobot_KI position initialization
wait_tran = [90 450 150];
ready_tran = [90 450 90];
ready_rota = [0 0 0];

% MoveRobot_ORI initialization
pos = [0 0 0];
bounds = [-0.3 -0.3  0  80;
    0.3  0.3  0  90];

% Start Robot
Robot_IP = '169.254.187.130';
Socket_conn = tcpip(Robot_IP,30010,'NetworkRole','server');
fclose(Socket_conn);
disp('Press Play on Robot...')
fopen(Socket_conn);
disp('Connected! Start random walk on UR5');
rota = get_pos(Socket_conn); % Read junk

% Go to wait tran & rota
for i = 1:100
    moverobot_Ki(Socket_conn, wait_tran, ready_rota)
end

net = resetState(net);
[sensor(1, :)] = read(dq, "OutputFormat", "Matrix");

for i=1:10

    % Read orientation & force
    disp('Start reading!');
    tic
    tim1(1)=0;
    flag = 2;
    for w = 1:200

        % Read
        [sensor1(flag,:,i)] = read(dq, "OutputFormat", "Matrix");
        [motion1(flag,:,i)] = [wait_tran ready_rota];

        % Interpolation
        tim1(flag,i) = toc;
        data2(flag,:,i) = sensor1(flag-1,1:16,i)+(sensor1(flag,1:16,i)-sensor1(flag-1,1:16,i))*(1/20)/(tim1(flag,i)-tim1(flag-1,i));%freq=25
        x{i} = (data2(flag,:,i)'- sensor1(2,1:16,i)') ./ sig;

        % Predict force & orientation
        [net,YPred_o] = predictAndUpdateState(net,x{i},'MiniBatchSize',1);
        pred(flag,:,i) = YPred_o .* sig2 + mu2;

        pred_ori(flag,:) = pred(flag,5:7,i);
        tar_fsum(flag) = sum(pred(flag,1:4,i));

        moverobot_Ki(Socket_conn, wait_tran, ready_rota);
        flag = flag + 1;
    end
    disp('----- End reading! -----');

    % Pick 1 orientation
    orientation = mean(pred_ori(33:166,:));
    for o = 1:2
        if orientation(o) > 0.3
            orientation(o) = 0.3;
        elseif orientation(o) < -0.3
            orientation(o) = -0.3;
        end
    end
    orientation(3) = 0;
    ori_to = vlinspace(ready_rota, orientation, 30);
    ori_back = vlinspace(orientation, ready_rota, 30);
    ori_list(i,:) = orientation;

    % Smoothen force profile
    pad(1:10,1) = repmat(tar_fsum(1),10,1);
    tar_fsum = [pad; tar_fsum'];
    for k=11:numel(tar_fsum)
        target(k) = mean(tar_fsum(k-10:k,1));
    end
    target = target(11:210);
    tarf(i,:) = target;

    pause(2);

    % Goes from wait to ready position
    go_path = vlinspace(wait_tran, ready_tran, 100);
    for t = 1:100
        moverobot_Ki(Socket_conn, go_path(t,:), ready_rota);
    end
    for j = 1:30 % Rotate to new orientation
        rota = ori_to(j,:);
        moverobot_Ki(Socket_conn, ready_tran, rota);
    end
    pause(5);

    % Press with predicted force & orientation
    tic
    tim2(1) = 0;
    flag = 2;
    trans = ready_tran;
    rota = orientation;
    for j = 1:200
        % Read data
        [sensor(flag,:,i)] = read(dq, "OutputFormat", "Matrix");
        [motion(flag,:,i)] = [trans rota];
        real_fsum(flag,i) = sum(sensor(flag,17:20,i));

        % Interpolation
        tim2(flag,i) = toc;
        data2(flag,:,i) = sensor(flag-1,1:16,i)+(sensor(flag,1:16,i)-sensor(flag-1,1:16,i))*(1/20)/(tim2(flag,i)-tim2(flag-1,i));%freq=25
        x{i} = (data2(flag,:,i)'- sensor(2,1:16,i)') ./ sig;

        % Predict force & orientation
        [net,YPred_o] = predictAndUpdateState(net,x{i},'MiniBatchSize',1);
        pred(flag,:,i) = YPred_o .* sig2 + mu2;

        pred_fsum(flag,i) = sum(pred(flag,1:4,i));
        err(flag) = tarf(i,flag-1) - pred_fsum(flag,i);
        err_list(flag,i) = err(flag);
        K = err(flag)*2;
        if err(flag) > 0.1
            trans = motion(flag,1:3,i) - K*[0,0,0.1];
        elseif  err(flag) < -0.5
            trans = motion(flag,1:3,i) - K*[0,0,0.1];
        else
            trans = motion(flag,1:3,i);
        end

        if trans(3)<bounds(1,4)||trans(3)>bounds(2,4)%%%%IMP
            trans = motion(flag,1:3,i);
        end
        moverobot_Ki(Socket_conn, trans, rota);
        flag = flag + 1;
    end
    
    % Move back
    for t = 1:30
        moverobot_Ki(Socket_conn, ready_tran, orientation);
    end
    for j = 1:30
        rota = ori_back(j,:);
        moverobot_Ki(Socket_conn, ready_tran, rota);
    end
    go_path = vlinspace(ready_tran, wait_tran, 100);
    for t = 1:100
        moverobot_Ki(Socket_conn, go_path(t,:), ready_rota);
    end
end
