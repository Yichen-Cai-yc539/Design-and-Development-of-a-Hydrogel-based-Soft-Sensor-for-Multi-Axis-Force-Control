h=figure;
ur = loadrobot("universalUR5");
config = homeConfiguration(ur);
config2 = homeConfiguration(ur);

h1 = show(ur,config);
for i=100:1:size(motion,1)
    for j=1:2

        if j==1
            trans = motion(100,1:3);
            rota = motion(i,4:6);
        else
            trans = motion(100,1:3);
            rota = pred(i,5:7);
        end

        rx = rota(1,1)+2.6;
        ry = rota(1,2)-1.7;
        rz = rota(1,3);

        th = sqrt(rx^2+ry^2+rz^2);
        u = (rota/th)';
        ux = u(1);
        uy = u(2);
        uz = u(3);
        xyz = [0 -uz uy;
            uz 0 -ux;
            -uy ux 0];

        c = cos(th);
        s = sin(th);
        t = 1 - c;

        Rm = c*eye(3) + t*(u*u') + s*xyz;
        Rm(1:3,1:3)=rotationVectorToMatrix([rx ry rz]);
        Rm(:,4) = trans;
        Rm(4,:) = [0 0 0 1];


        Tr = Rm;
        pyTr = mat2np(Tr);
        joints = pyrunfile("ur5Kine.py", "th", desired_pos = pyTr);
        mat = np2mat(joints);
        joint_pos(:,j) = mat(:,5);


        if j==1
            if any(isnan(joint_pos(:,j))) ~=1
                for k=1:6
                    config(k).JointPosition = joint_pos(k,j);
                end
            end
        else
            if any(isnan(joint_pos(:,j))) ~=1
                for k=1:6
                    config2(k).JointPosition = joint_pos(k,j);
                end
            end
        end


    end
    % lastJoint(i,:) = mat(6,:);
    delete(h1)
    delete(h2)

    h1 = show(ur,config, 'Visuals','on');
    alpha(0.5);
    hold on
    h2 = show(ur,config2,'Visuals','on');
    alpha(0.5)
    view(135, 30)
    xlim([-0.4 0])
    ylim([-1 -0.6])
   zlim([0 0.4])

%         xlim([-2 2])
%         ylim([-2 2])
%         zlim([0 2])
    pause(0.02)

end

%
% [configSol,solInfo] = ik(endeffector,pose,weights,config);
%
%

% ik = inverseKinematics('RigidBodyTree',ur);
%
%  weights = motion(1,:);
%  initialguess = ur.homeConfiguration;
%  weights = [0 0 0 1 1 1];
% % %Calculate the joint positions using the ik object.
% %
% [configSoln,solnInfo] = ik('base_link',Tr,weights,initialguess);
%
% show(ur,configSoln);
