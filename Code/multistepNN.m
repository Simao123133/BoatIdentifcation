%Script that creates the multistep feedforward network model
clear
load('raw_data.mat')
start = 1;
finish = length(U.signals.values)-1;
uf = lowpass(medfilt1(U.signals.values(start:finish),3),1,10);
wf = lowpass(medfilt1(W.signals.values(start:finish),3),1,10);
prf = lowpass(diff(medfilt1(Pitch.signals.values(start:finish+1),5))/0.1,1,10); %pitch rate filtered
pitchf = lowpass(medfilt1(Pitch.signals.values(start:finish),3),1,10);
zf = lowpass(Height.signals.values(start:finish),1,10);
vf = lowpass(medfilt1(V.signals.values(start:finish),3),1,10);
rrf = lowpass(diff(medfilt1(Roll.signals.values(start:finish+1),3))/0.1,1,10); %roll rate filtered
rf = lowpass(medfilt1(gyrZ.signals.values(start:finish),15),1,10);
rollf = medfilt1(Roll.signals.values(start:finish),3);
yawf = lowpass(medfilt1(Yaw.signals.values(start:finish),10),1,10);
LFf =  lowpass(medfilt1(Left_foil.signals.values(start:finish),3),1,10);
RFf = lowpass(medfilt1(Right_foil.signals.values(start:finish),3),1,10);
ReFf = Rear_foil.signals.values(start:finish);
Rpmsf = medfilt1(lowpass(Motor.signals.values(start:finish),2,10),6);
Rudderf = lowpass(medfilt1(Rudder.signals.values(start:finish),10),1,10);

i=1;
for j=1:length(uf)/10
   inputs(j,1:9) = [uf(i) wf(i) prf(i) pitchf(i) zf(i) vf(i) rrf(i) rf(i) rollf(i)];
   for k=1:10
      inputs(j,4*k-3+9:4*k+9) = [LFf(i) RFf(i) Rpmsf(i) Rudderf(i)];
      i=i+1;
      outputs(j,9*k-8:9*k) = [uf(i) wf(i) prf(i) pitchf(i) zf(i) vf(i) rrf(i) rf(i) rollf(i)];
   end
end

trainFcn = 'trainlm';  % Levenberg-Marquardt backpropagation.

net = feedforwardnet(20,trainFcn);

% Prepare the Data for Training and Simulation
% The function PREPARETS prepares timeseries data for a particular network,
% shifting time by the minimum amount to fill input states and layer
% states. Using PREPARETS allows you to keep your original time series data
% unchanged, while easily customizing it for networks with differing
% numbers of delays, with open loop or closed loop feedback modes.

% Setup Division of Data for Training, Validation, Testing
net.divideParam.trainRatio = 70/100;
net.divideParam.valRatio = 15/100;
net.divideParam.testRatio = 15/100;
net.divideFcn = 'divideblock';
% Train the Network
[net,tr] = train(net,inputs', outputs');

% Test the Network
y = net(inputs');
results = y';
titles = ["u",'w','dPitch','pitch','z','v','dRoll','dYaw','roll'];
labels =["m/s",'m/s','deg/s','deg','m','m/s','deg/s','deg/s','deg'];
figure(4)
j=1;
results = outputs(tr.testInd(1)-1,:);
for i=1:100
    sample = tr.testInd(i);
    y = net([results(82:90) inputs(sample,10:end)]');
    results = y';
    for h=1:10
        res(j,1) = outputs(sample,9*h-8);
        res(j,2) = outputs(sample,9*h-7);
        res(j,3) = outputs(sample,9*h-6);
        res(j,4) = outputs(sample,9*h-5);
        res(j,5) = outputs(sample,9*h-4);
        res(j,6) = outputs(sample,9*h-3);
        res(j,7) = outputs(sample,9*h-2);
        res(j,8) = outputs(sample,9*h-1);
        res(j,9) = outputs(sample,9*h);
        resy(j,1) = results(9*h-8);
        resy(j,2) = results(9*h-7);
        resy(j,3) = results(9*h-6);
        resy(j,4) = results(9*h-5);
        resy(j,5) = results(9*h-4);
        resy(j,6) = results(9*h-3);
        resy(j,7) = results(9*h-2);
        resy(j,8) = results(9*h-1);
        resy(j,9) = results(9*h);
        j=j+1;
    end
end
for i=1:9
subplot(3,3,i);
plot(0:0.1:length(res)/10-0.1,res(:,i),0:0.1:length(res)/10-0.1, resy(:,i));
title(titles(i))
xlabel('Time [s]')
ylabel(labels(i))
legend('Real','NN') 
end



%save('net.mat','net');



