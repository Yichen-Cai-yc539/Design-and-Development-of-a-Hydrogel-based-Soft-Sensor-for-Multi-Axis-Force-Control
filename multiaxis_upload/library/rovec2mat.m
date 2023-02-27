function Rm = rovec2mat(rota)

% Rotational Vector -> Rotational Matrix
rx = rota(1,1);
ry = rota(1,2);
rz = rota(1,3);
th = sqrt(rx^2+ry^2+rz^2); % Find theta

u = (rota/th)';
ux = u(1);
uy = u(2);
uz = u(3);
xyz = [0 -uz uy; % Find matrix
    uz 0 -ux;
    -uy ux 0];

c = cos(th);
s = sin(th);
t = 1 - c;

Rm = c*eye(3) + t*(u*u') + s*xyz; % Rotational matrix

% Rotational Matrix -> Transformation Matrix
% Rm(:,4) = trans;
% Rm(4,:) = [0 0 0 1];
% Tr = Rm;

% InvKine -> Last Joint Pos
%     pyTr = mat2np(Tr);
%     joints = pyrunfile("ur5Kine.py", "th", desired_pos = pyTr);
%     mat = np2mat(joints);
%     lastJoint(i,:) = mat(6,:);

end