% Robot Initialization. Robot_IP = IP on robot; remember to check IP on
% robot dashborad is equal to that in ipconfig (laptop IP); Check ethernet
% connection; Check firewall setting; Try restart; Check 'base' mode is on.

% Soft sensor initialization
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

% Start moving & data acquisition
for k = 1:25

    % Generate random walk data
    walk = random(bounds(:,1:3), 31, 3);
    tx([1:size(walk,1)],1) = ready_tran(1);
    ty([1:size(walk,1)],1) = ready_tran(2);
    tz = random(bounds(:,4), 31, 1);
    Tran_array = [ready_tran; [tx ty tz]];
    walk = [[0 0 0]; walk];

    flag = 1;
    text = sprintf('Collecting batch no. %d', k);
    disp(text);

    tic

    for t = 2:size(Tran_array, 1)
        message = sprintf('Press no. %d', t-1);
        disp(message);
        n = randi(50);
        m = randi(50);
        total = 150 + randi(50)+30;
        get = 0;
        for j = 1:total
            if j < 60 % Initialize tran & rota
                trans = ready_tran;
                rota = ready_rota;
                mode = 0;
            elseif j == 60 % Rotate to new orientation
                wk = walk(t,:);
                mode = 1;
            elseif j < 70+n % Moves down j = 70 - 120; duration 10 - 60
                
                if get == 0
%                     disp('try')
                    emp = [];
                    while length(emp) < 6
                        emp = get_pos(Socket_conn);
                    end
%                     disp('trydone')
                end
                rota = emp(4:6);
                get = 1;
                path_z = linspace(ready_tran(3), Tran_array(t,3), 10+n);
                trans = [Tran_array(t,[1:2]) path_z(j-60)];
                mode = 0;
            elseif j < 120+m % Stays down j = 71 - 170; duration 0 - 100
                trans = Tran_array(t,:);
                mode = 0;
            elseif j < total % Moves up j = 72 - 200; duration 10 - 60
                path_z = linspace(Tran_array(t,3), ready_tran(3), total-119-m);
                trans = [Tran_array(t,[1:2]) path_z(j-(119+m))];
                mode = 0;
            elseif j == total
                wk = invert(walk(t,:));
                mode = 1;
            end
            if mode == 0
                moverobot_Ki(Socket_conn, trans, rota);
            elseif mode == 1
                moverobot_ori(Socket_conn, pos, wk);
                pause(8)

            end
            
            % Read data
            [sensor(flag, :, k)] = read(dq, "OutputFormat", "Matrix");
            [motion(flag, :, k)] = [trans rota];

            tim(flag, k) = toc;

            flag = flag + 1;
        end
    end
%     disp('Saving');
    save 0_25batch31press.mat
    disp('Saved');
    rp = randi(60);
    p = rp + 30;
    pause(p)
end

disp('Random walk on UR5 finished')
save 0_25batch31press.mat