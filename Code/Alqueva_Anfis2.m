%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file imports raw data measured in alqueva. And derives 9 Diferent
% Miso Takagi-Sugeno Systems with optimized paramters via use of a neural
% network.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Load data

load('raw_data.mat')
start = 1;
finish = length(Motor.signals.values);
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

u = um;
w = wm;
q = qm;
pitch = pitchm;
z = zm;
v = vm;
p = pm;
r = rm;
roll = rollm;
LF = LFm;
RF = RFm;
Rudder = Rudderm;
Rpms = Rpmsm;



%% %%%%%%%%%%%%%%%%%%%%%% Divide data set %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

n = 9; % single output selection
outputs = [u,w,q,pitch,z,v,p,r,roll];
inputs = [LF,RF,Rudder,Rpms,outputs(:,n)];

% Delay input
inputs = inputs(1:end-1,:);
outputs = outputs(2:end,:);
t = 0:0.1:length(LF)/10-0.1;


% remore outliers from rudder angle
max_lim = 200;
min_lim = -200;
x = inputs(:,3);
x(x>max_lim) = max_lim;
x(x<min_lim) = min_lim;
inputs(:,3) = x;

% normalize data set
for i=1:5
    range = max(inputs(:,i)) - min(inputs(:,i));
    x =(inputs(:,i) - min(inputs(:,i))) / range;
    inputs(:,i) = 2*x-1; 
end

for i=1:9
    range = max(outputs(:,i)) - min(outputs(:,i));
    x=(outputs(:,i) - min(outputs(:,i))) / range;
    outputs(:,i)=2*x-1;    
end

% Train set
start = 7650;
% fim = 9460;
fim=11200;
inputs_train = inputs(start:fim,:);
outputs_train = outputs(start:fim,:);

% test set
start = 20142-6042;
fim = 20142;

inputs_test = inputs(start:fim,:);
outputs_test = outputs(start:fim,:);


%% Train Model
tic
% data formating
data_anfis = [inputs_train outputs_train(:,n)];

% Init model
genOpt = genfisOptions('GridPartition');
genOpt.NumMembershipFunctions = 3;
genOpt.InputMembershipFunctionType = 'gbellmf';
inFIS = genfis(inputs_train,outputs_train(:,n),genOpt);

% Training Options
opt = anfisOptions('InitialFIS',inFIS,'EpochNumber',10);
opt.DisplayANFISInformation = 1;
opt.DisplayErrorValues = 1;
opt.DisplayStepSize = 1;
opt.DisplayFinalResults = 1;

% Train model
outFIS = anfis(data_anfis,opt);
toc

% Plot results train
anfis_out = evalfis(outFIS,inputs_train);
figure,
plot(outputs_train(:,n)); hold on
plot(anfis_out); hold off
legend('Training Data','ANFIS Output')


for i=1:5
    max_lim = max(inputs_train(:,i));
    min_lim = min(inputs_train(:,i));
    x = inputs_test(:,i);
    x(x>max_lim) = max_lim;
    x(x<min_lim) = min_lim;
    inputs_test(:,i) = x;
end

anfis_out_test = evalfis(outFIS,inputs_test);
% anfis_out_test = evalfis(anfis_z,inputs_test);
figure,
plot(outputs_test(:,n)); hold on
plot(anfis_out_test); hold off
legend('Test Data','ANFIS Output')

%%

figure,
plotmf(anfis_u,'input',5)
title('Input 1')

