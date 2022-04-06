%Generate Narxnet from data
clear
load('raw_data.mat')
start = 1;
finish = length(U.signals.values);
ua = medfilt1(U.signals.values(start:finish),10);
wa = medfilt1(W.signals.values(start:finish),10);
qa = medfilt1(gyrY.signals.values(start:finish),50);
pitcha = medfilt1(Pitch.signals.values(start:finish),10);
za = medfilt1(Height.signals.values(start:finish),10);
va = medfilt1(V.signals.values(start:finish),10);
pa = medfilt1(gyrX.signals.values(start:finish),50);
ra = medfilt1(gyrZ.signals.values(start:finish),50);
rolla = medfilt1(Roll.signals.values(start:finish),10);
LFa = Left_foil.signals.values(start:finish);
RFa = Right_foil.signals.values(start:finish);
ReFa = Rear_foil.signals.values(start:finish);
Rpmsa = medfilt1(Motor.signals.values(start:finish),10);
Ruddera = medfilt1(Rudder.signals.values(start:finish),10);
load('raw_data_montargil.mat')
start = 1;
finish = 1 + 4500/0.1;
um = medfilt1(U.signals.values(start:finish),10);
wm = medfilt1(W.signals.values(start:finish),10);
qm = medfilt1(gyrY.signals.values(start:finish),50);
pitchm = medfilt1(Pitch.signals.values(start:finish),10);
zm = medfilt1(Height.signals.values(start:finish),10);
vm = medfilt1(V.signals.values(start:finish),10);
pm = medfilt1(gyrX.signals.values(start:finish),50);
rm = medfilt1(gyrZ.signals.values(start:finish),50);
rollm = medfilt1(Roll.signals.values(start:finish),10);
LFm = Left_foil.signals.values(start:finish);
RFm = Right_foil.signals.values(start:finish);
ReFm = Rear_foil.signals.values(start:finish);
Rpmsm = medfilt1(Motor.signals.values(start:finish),10);
Rudderm = medfilt1(Rudder.signals.values(start:finish),10);

splitm = round((finish-start)/2);

u = [ua;um];
w = [wa;wm];
q = [qa;qm];
pitch = [pitcha;pitchm];
z = [za;zm];
v = [va;vm];
p = [pa;pm];
r = [ra;rm];
roll = [rolla;rollm];
LF = [LFa;LFm];
RF = [RFa;RFm];
Rudder = [Ruddera;Rudderm];
Rpms = [Rpmsa;Rpmsm];

inputs = [LFa,RFa,Ruddera,Rpmsa];
outputs = [ua,wa,qa,pitcha,za,va,pa,ra,rolla];
t = 0:0.1:length(LFa)/10-0.1;


titles = ["u",'w','dpitch','pitch','z','v','droll','dyaw','roll'];
labels = ["m/s",'m/s','deg/s','deg','cm','m/s','deg/s','deg/S','deg'];
figure(1)
for i=1:9
subplot(3,3,i)
plot(t,outputs(:,i));
xlabel('time')
ylabel(labels(i));
title(titles(i))
end
% Solve an Autoregression Problem with External Input with a NARX Neural Network
% Script generated by Neural Time Series app
% Created 22-Nov-2021 17:46:54
%
% This script assumes these variables are defined:
%
%   inputs - input time series.
%   outputs - feedback time series.

X = tonndata(inputs,false,false);
T = tonndata(outputs,false,false);

% Choose a Training Function
% For a list of all training functions type: help nntrain
% 'trainlm' is usually fastest.
% 'trainbr' takes longer but may be better for challenging problems.
% 'trainscg' uses less memory. Suitable in low memory situations.
trainFcn = 'trainlm';  % Levenberg-Marquardt backpropagation.

% Create a Nonlinear Autoregressive Network with External Input
inputDelays = 1;
feedbackDelays = 1;
hiddenLayerSize = [20 20 20 20];
net = narxnet(inputDelays,feedbackDelays,hiddenLayerSize,'open',trainFcn);

% Prepare the Data for Training and Simulation
% The function PREPARETS prepares timeseries data for a particular network,
% shifting time by the minimum amount to fill input states and layer
% states. Using PREPARETS allows you to keep your original time series data
% unchanged, while easily customizing it for networks with differing
% numbers of delays, with open loop or closed loop feedback modes.
[x,xi,ai,t] = preparets(net,X,{},T);

% Setup Division of Data for Training, Validation, Testing
net.divideParam.trainRatio = 70/100;
net.divideParam.valRatio = 15/100;
net.divideParam.testRatio = 15/100;
net.divideFcn = 'divideblock';

% Train the Network
[net,tr] = train(net,x,t,xi,ai);

% Test the Network
y = net(x,xi,ai);
e = gsubtract(t,y);
performance = perform(net,t,y);


% Plot
% Uncomment these lines to enable various plots.
%figure, plotperform(tr)
%figure, plottrainstate(tr)
%figure, ploterrhist(e)
%figure, plotregression(t,y)
%figure, plotresponse(t,y)
%figure, ploterrcorr(e)
%figure, plotinerrcorr(x,e)

% Closed Loop Network
% Use this network to do multi-step prediction.
% The function CLOSELOOP replaces the feedback input with a direct
% connection from the output layer.
netc = closeloop(net);
netc.name = [net.name ' - Closed Loop'];
[xc,xic,aic,tc] = preparets(netc,X,{},T);
yc = netc(xc,xic,aic);
closedLoopPerformance = perform(net,tc,yc);

% Step-Ahead Prediction Network
% For some applications it helps to get the prediction a timestep early.
% The original network returns predicted y(t+1) at the same time it is
% given y(t+1). For some applications such as decision making, it would
% help to have predicted y(t+1) once y(t) is available, but before the
% actual y(t+1) occurs. The network can be made to return its output a
% timestep early by removing one delay so that its minimal tap delay is now
% 0 instead of 1. The new network returns the same outputs as the original
% network, but outputs are shifted left one timestep.
figure(2)
res = cell2mat(y)';
start =1;
finish = 1200;
t = 0:0.1:length(y)/10-0.1;
for i=1:9
subplot(3,3,i);
plot(t(tr.testInd(start:finish)),res(tr.testInd(start:finish),i),t(tr.testInd(start:finish)),outputs(tr.testInd(start:finish),i));
title(titles(i))
xlabel('time')
ylabel(labels(i))
legend('NN','Real') 
end

X = tonndata(inputs(tr.testInd,:),false,false);
T = tonndata(outputs(tr.testInd,:),false,false);
[xc,xic,aic,tc] = preparets(netc,X,{},T);
yc = netc(xc,xic,aic);
start = 1;
finish = length(tr.testInd);
res = cell2mat(yc)';
t = 0:0.1:length(yc)/10-0.1;
figure(3)
for i=1:9
subplot(3,3,i);
plot(t(start:finish),res(start:finish,i),t(start:finish),outputs(tr.testInd(start:finish),i));
title(titles(i))
xlabel('time')
ylabel(labels(i))
legend('NN','Real') 
end

figure(4)
ititles = ["LF",'RF','Rudder','Motor'];
ilabels = ["deg",'deg','deg','rpms'];
for i=1:4
subplot(2,3,i);
plot(t(start:finish),inputs(tr.testInd(start:finish),i));
title(ititles(i))
xlabel('time')
ylabel(ilabels(i))
end


X = tonndata([0 0 0 7000].*ones(100,4),false,false);
T = tonndata([7 0 0 0 65 0 0 0 0].*ones(100,9),false,false);
[x,xi,ai,t] = preparets(netc,X,{},T);
testing = netc(xc,xic,aic);
testr = cell2mat(testing)';

[sysName,netName] = gensim(netc,'InputMode','Workspace',...
	'OutputMode','WorkSpace','SolverMode','Discrete');

setsiminit(sysName,netName,netc,xi,ai,1);

for i=1:9
   MSE(i) =  sum((res(tr.testInd(start:finish),i) - outputs(tr.testInd(start:finish),i)).^2)/length(t(tr.testInd(start:finish)));
end



