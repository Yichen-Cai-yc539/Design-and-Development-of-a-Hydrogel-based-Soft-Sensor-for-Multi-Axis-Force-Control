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
ready_tran = [89.11 449.81 92];
ready_rota = [0 0 0];

% MoveRobot_ORI initialization
pos = [0 0 0];
bounds = [-0.3 -0.3  0  80;
    0.3  0.3  0  92];

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
force_range = 10;

tic

tim1(1)=0;
tim2(1)=0;


[sensor(1, :)] = read(dq, "OutputFormat", "Matrix");

for i=1:1

    walk = [0.107821606112205	0.0930588023843044	0]; %random(bounds(:,1:3), 1, 3);
    t = 0:0.05:800;
    y = chirp(t,0,800,0.3);
    tarf = 12.5 + 2*(1+y);

    tic
    get = 0;
    flag = 2;
    for j = 1:12060
        if j < 30 % Initialize tran & rota
            trans = ready_tran;
            rota = ready_rota;
            mode = 0;
        elseif j == 30 % Rotate to new orientation
            wk = walk;
            mode = 1;
        elseif j <= 12031 % Enter control loop
            if get == 0
                emp = [];
                while length(emp) < 6
                    emp = get_pos(Socket_conn);
                end
            end
            get = 1;
            rota = emp(4:6);
            mode = 2;
        elseif j < 12060 % Move up
            mode = 0;
        elseif j == 12060
            wk = invert(walk);
            mode = 1;
        end

        if flag < 32
            target(flag) = tarf(1);
        else
            target(flag) = tarf(flag-31);
        end

        [sensor(flag,:,i)] = read(dq, "OutputFormat", "Matrix");
        [motion(flag,:,i)] = [trans rota];
        real_sum(flag) = sum(sensor(flag,17:20,i));


        tim2(flag,i) = toc;
        data2(flag,:,i) = sensor(flag-1,1:16,i)+(sensor(flag,1:16,i)-sensor(flag-1,1:16,i))*(1/20)/(tim2(flag,i)-tim2(flag-1,i));%freq=25
        x{i} = (data2(flag,:,i)'- sensor(2,1:16,i)') ./ sig;
        [net,YPred_o] = predictAndUpdateState(net,x{i},'MiniBatchSize',1);
        pred(flag,:,i) = YPred_o .* sig2 + mu2;
        pred_sum(flag) = sum(pred(flag,1:4,i));
        err(flag) = target(flag) - pred_sum(flag);

        if mode == 0
            moverobot_Ki(Socket_conn, ready_tran, rota);
        elseif mode == 1
            moverobot_ori(Socket_conn, pos, wk);
            pause(8)
        elseif mode == 2

            K = err(flag)*0.4;
            if err(flag) > 0.05
                trans = motion(flag,1:3,i) - K*[0,0,0.1];
            elseif  err(flag) < -0.05
                trans = motion(flag,1:3,i) - K*[0,0,0.1];
            else
                trans = motion(flag,1:3,i);
            end

            if trans(3)<bounds(1,4)||trans(3)>bounds(2,4)%%%%IMP
                trans = motion(flag,1:3,i);
            end
            moverobot_Ki(Socket_conn, trans, rota);
        end

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



%         if rem(floor(i/50),2)==0
%            des(i+1)=1.6;%des(i)-0.003;
%         else
%            des(i+1)=1.1;%des(i)+0.003;
%         end
%     des(i+1) = des(i);%1.3-0.2*sin(i/50);
%
%     des(i+1)=1.55-0.4*abs(sin(i/50));
%     des(i+1)=1.41+0.2*(sin(i/(400-(380/4000)*(i-1))));
%
%     des(i+1)=1.61-0.25*abs(sin(i/100))-0.25*abs(sin(i/70));
%     des(i+1)=1.61-0.4*floor(abs(2*sin(i/100))-0.00001);
%     des(i+1)=1.25;




%freq=freq-22/20000;
%i

% Functions

