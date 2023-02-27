% Goal_Pose should be in mm and Orientation the rotation vector
function P_new = moverobot_ori(t,Goal_Pose,Orientation)
a=1;
if nargin == 1
    error('error; not enough input arguments')
elseif nargin == 2
    P = Goal_Pose;
elseif nargin == 3
    P = [Goal_Pose,Orientation];
end
P(1:3) = P(1:3) * 0.001; % Converting to meter
P_char = ['(',num2str(3),',',...
    num2str(P(1)),',',...
    num2str(P(2)),',',...
    num2str(P(3)),',',...
    num2str(P(4)),',',...
    num2str(P(5)),',',...
    num2str(P(6)),',',...
    num2str(0.01),',',...%acc
    num2str(0.1),',',...%vel
    num2str(0),',',...%t
    num2str(0),',',...%r
    num2str(0),...%r
    num2str(0),...%r
    ')'];
success = '0';
while strcmp(success,'0')
    
    fprintf(t,P_char);

    while a ~= 1
        if t.BytesAvailable > 0
            a = fscanf(t,'%c',t.BytesAvailable);
            a = str2num(a);
        else
            a = 0;
        end

        %while t.BytesAvailable==0
        % a= t.BytesAvailable;
    end
    %    success  = fscanf(t,'%c',t.BytesAvailable);
    %  % pause(0.05)
    success ='1';


end
if ~strcmp(success,'1')
    error('error sending robot pose')
end
%pause(0.5)
P_new =0;
% P_new = readrobotpose(t);