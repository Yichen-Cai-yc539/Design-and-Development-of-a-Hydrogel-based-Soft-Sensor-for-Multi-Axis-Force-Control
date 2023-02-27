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
% GND force strain gauge initialization
for i = 0:1:3
    chn = sprintf('ai%d', i);
    chi = addinput(dq, 'Dev4', chn, 'Voltage');
    chi.TerminalConfig = "SingleEnded";
end

% Set bounds: vertical
% ready_tran = [90 450 250];
% bounds = [120 480 280;
%     60 420 220];
% ready_rota = [0 0 0];

% Set bounds: horizontal
ready_tran = [90 450 250];
bounds = [120 480 280;
    60 420 220];
ready_rota = [1.4 0 0];

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

% In loop: read sensor, stay still, predict force vector
N = 10000000000000000000000000000;

tic

h=figure;
tim1(1)=0;
tim2(1)=0;

h1=quiver3(0,0,0,0,0,0);

for i=2:N

    [sensor(i, :)] = read(dq, "OutputFormat", "Matrix");
    [motion(i, :)] = [ready_tran ready_rota];

    % Interpolation
    tim2(i)=toc;
    data2(i,:)=sensor(i-1,1:16)+(sensor(i,1:16)-sensor(i-1,1:16))*(1/20)/(tim2(i)-tim2(i-1));%freq=25
    x{1}=(data2(i,1:16)'-sensor(2,1:16)')./sig;

    % Predict force & Orientation
    [net,YPred_o]= predictAndUpdateState(net,x{1},'MiniBatchSize',1);
    pred(i,:)=YPred_o.*sig2+mu2;
    force(i,:) = pred(i,1:4);
    orientation(i,:) = pred(i,5:7);

    % Calculate Direction of Force
    rota_o = orientation(i,:);
    rota = ready_rota;
    Fo = [0 0 sum(force(i,:))-11.85];
    F(i,:) = f2delt(rota_o, rota, Fo);

    % Move robot
    move = 0;
    if i > 50
        nextT_list(1:50,:) = repmat(ready_tran, 50, 1);
%         p = [50 50 5]; % Vertical parameter
        p = [50 5 50]; % Horizontal parameter
        bfactor = 0.01;

        dT(i,:) = p .* mean(F(i-1:i,:));
        nextT_list(i,:) = ready_tran + dT(i,:);
        nextT = nextT_list(i,:);

        for nt = 1:3
            if nextT(nt) < bounds(1,nt) && nextT(nt) > bounds(2,nt)
                move = 1;
            else
                dT(i,:) = bfactor * (ready_tran - nextT_list(i-1,:));
                nextT_list(i,:) = nextT_list(i-1,:) + dT(i,:);
                nextT = nextT_list(i,:);
                move = 1;
            end
        end


        if move == 1
            moverobot_Ki(Socket_conn, nextT, ready_rota);
        end
    end
    %     moverobot_Ki(Socket_conn, ready_tran, ready_rota);
end

for i=1:size(F,1)
    % Plot
    delete(h1);
    h1 = quiver3(0,0,0, F(i,1),F(i,2),F(i,3), 'b', 'LineWidth', 1);
    axis equal
    xlim([-0.6 0.6]);
    ylim([-0.6 0.6]);
    zlim([-0.5 1.5]);
    view(-45, 10);

    pause(0.02);
end