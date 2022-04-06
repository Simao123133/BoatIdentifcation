% Perform filtering on the raw data
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
u = U.signals.values(start:finish);
w = W.signals.values(start:finish);
q = gyrY.signals.values(start:finish);
pr = diff(Pitch.signals.values(start:finish+1))/0.1;
pitch = Pitch.signals.values(start:finish);
z = Height.signals.values(start:finish);
v = V.signals.values(start:finish);
rr = diff(Roll.signals.values(start:finish+1))/0.1;
p = gyrX.signals.values(start:finish);
r = gyrZ.signals.values(start:finish);
roll = Roll.signals.values(start:finish);
yaw = Yaw.signals.values(start:finish);
LF = Left_foil.signals.values(start:finish);
RF = Right_foil.signals.values(start:finish);
ReF = Rear_foil.signals.values(start:finish);
Rpms = Motor.signals.values(start:finish);
Rudder = Rudder.signals.values(start:finish);

inputs = [LF,RF,Rudder,Rpms];
outputs = [u,w,pr,pitch,z,v,rr,r,roll,yaw];
inputsf = [LFf,RFf,Rudderf,Rpmsf];
outputsf = [uf,wf,prf,pitchf,zf,vf,rrf,rf,rollf,yawf];

start = 5000;
finitto = 5200;
figure(1)
titles = ["LF",'RF','Rudder','Motor'];
labels = ["deg",'deg','deg','Rpms'];
for i=1:4
subplot(2,2,i)
plot(0:0.1:(finitto-start)/10,inputs(start:finitto,i),0:0.1:(finitto-start)/10,inputsf(start:finitto,i));
title(titles(i))
xlabel('Time [s]')
ylabel(labels(i))
legend('raw','filtered')
end
titles = ["u",'w','dpitch','pitch','z','v','droll','dyaw','roll','yaw'];
labels = ["m/s",'m/s','deg/s','deg','cm','m/s','deg/s','deg/s','deg','deg'];
figure(3)
for i=1:9
subplot(3,3,i)
plot(0:0.1:(finitto-start)/10,outputs(start:finitto,i),0:0.1:(finitto-start)/10,outputsf(start:finitto,i));
title(titles(i))
xlabel('Time [s]')
ylabel(labels(i))
legend('raw','filtered')
end



