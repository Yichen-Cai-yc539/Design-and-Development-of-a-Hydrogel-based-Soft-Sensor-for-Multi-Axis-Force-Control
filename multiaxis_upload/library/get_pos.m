% Goal_Pose should be in mm and Orientation the rotation vector
function P_new = get_pos(t)



P_char = ['(',num2str(10),',',...
    num2str(1),',',...
    num2str(1),',',...
    num2str(1),',',...
    num2str(1),',',...
    num2str(1),',',...
    num2str(1),',',...
    num2str(0.01),',',...%acc
    num2str(0.1),',',...%vel
    num2str(0),',',...%t
    num2str(0),',',...%r
    num2str(1),...%r
    ')'];
success = '0';
while strcmp(success,'0')
    
    fprintf(t,P_char);
    pause(0.001)
    if t.BytesAvailable > 0
        a = fscanf(t,'%c',t.BytesAvailable);
    else
        a = '';
    end
        %b = t.BytesAvailable;
%     while t.BytesAvailable==0
%           r=1
%        a = t.BytesAvailable;
%     end
 %    success  = fscanf(t,'%c',t.BytesAvailable);
 %  % pause(0.05)
    success ='1';
    
    
end

%pause(0.5)
P_new = a(3:end-1);
P_new = str2num(string(P_new));
% P_new = readrobotpose(t);