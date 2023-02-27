% % Resample
% for i=1:25
%     %     for j = 1:size(motion, 1)
%     %         mx = motion(j,4);
%     %         my = motion(j,5);
%     %         mz = motion(j,6);
%     %         [azimuth,elevation,r] = sph2cart(mx,my,mz);
%     %         motion_sph(j,:,i) = [azimuth,elevation,r];
%     %     end
% 
%     thresh1 = 2.88;
%     zero = [0 0 0];
%     for j = 1:size(sensor(:,:,i), 1)
%         if sensor(j,17,i) < thresh1
%             motion(j,4:6,i) = zero;
%         end
%     end
% 
%     rawIn{i} =resample(squeeze([sensor(1:end,1:16,i)]),squeeze(tim(1:end,i)),20,'pchip');
%     rawOut{i} =resample(squeeze([sensor(1:end, 17:20,i) motion(:, 4:6,i)]),squeeze(tim(1:end,i)),20,'pchip');% motion_sph(:, :,i)
% 
%     cropIn{i}=rawIn{i}(11:15400,:)';
%     cropOut{i}=rawOut{i}(11:15400,:)';
% 
% end


% Sets for training (no 2,15,29,42,64,81,98,110)
tr=[1:11,13:24];
%tr=[1:25];

% Normalization
x=cropIn';
y=cropOut';
XTest=cropIn';
YTest=cropOut';
%YTest=cell2mat(vel')';

mu = mean([x{i}],2); %{:}
sig = std([x{i}],0,2); %{:}
mu2 = mean([y{i}],2); %{:}
sig2 = std([y{i}],0,2); %{:}

for i = 1:numel(x)
    XTest{i} = (XTest{i}-x{i}(:,1)) ./ sig;
    x{i} = (x{i} - x{i}(:,1)) ./ sig;
    %  x{i} = (x{i}:,1) ./ sig;

end
for i = 1:numel(y)
    y{i} = (y{i} - mu2) ./ sig2;
    %  x{i} = (x{i}) ./ sig;
    YTest{i} = (YTest{i}-mu2) ./ sig2;
end
% y = y';
% x = x';
% YTest = YTest';
% XTest = XTest';

% LSTM
numResponses = size(y{i},1);
%numResponses = size(y,1);
featureDimension = size(x{i},1);
numHiddenUnits = 20;
layers = [
    sequenceInputLayer(featureDimension)
    % lstmLayer(numHiddenUnits,'OutputMode','sequence')
    lstmLayer(numHiddenUnits,'OutputMode','sequence')
    dropoutLayer(0.3)
    lstmLayer(numHiddenUnits,'OutputMode','sequence')
    % lstmLayer(numHiddenUnits)
    fullyConnectedLayer(numResponses)
    % tanhLayer
    regressionLayer];

% Training
maxEpochs = 20000;
miniBatchSize = 512;

options = trainingOptions('adam', ...
    'MaxEpochs',maxEpochs, ...
    'MiniBatchSize',miniBatchSize, ...
    'InitialLearnRate',0.0002*8, ... %0.002
    'ValidationData',{XTest([2,25]),YTest([2,25])}, ...%
    'LearnRateDropFactor',0.99, ...
    'LearnRateDropPeriod',100, ...
    'GradientThreshold',0.01*1000, ...
    'Shuffle','every-epoch', ...
    'Plots','training-progress',...
    'L2Regularization',0.01,...
    'ExecutionEnvironment', 'GPU',...
    'Verbose',1);
%
[net,info] = trainNetwork(x(tr),y(tr),layers,options);

%XTest=inp_all;
%YTest=out;

% Predict
[net,YPred_o]= predictAndUpdateState(net,XTest,'MiniBatchSize',1);

YPred = predict(net,x,'MiniBatchSize',1);

% asd=(YTest{20}');
%
% asd2=(YPred{20}');
%
%
% asd3=(YPred_o{2});
%
% plot(asd)
% hold on
% plot(asd2)

%
% for i=1:421
%     pos=pos_all{i, 1}(:,1:end);
%     if pos(1,500)==NaN
%         i
%     end
%    % scatter3(pos(1,:),pos(2,:),pos(3,:))
%     hold on
%
% end
