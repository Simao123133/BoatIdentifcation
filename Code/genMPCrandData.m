% Generates MPC random data to train the NN 
clear
load('trim_op_fixed_v')
load('BoatMatrices');
C = [1 0 0 0 0 0 0 0 0;
     0 0 0 0 1 0 0 0 0;
     0 0 0 1 0 0 0 0 0;
     0 0 0 0 0 0 0 0 1;
     0 0 0 0 0 0 0 1 0];
D = zeros(5,5);

cont_Boat = ss(A,B,C,D);
Ts = 0.1;
Boat = c2d(cont_Boat,Ts);
%% create MPC controller object with sample time
mpcobj = mpc(Boat);
%% specify prediction horizon
mpcobj.PredictionHorizon = 20;
%% specify control horizon
mpcobj.ControlHorizon = 20;
%% specify nominal values for inputs and outputs
mpcobj.Model.Nominal.U = [0;0;0;0;0];
mpcobj.Model.Nominal.Y = [0;0;0;0;0];
%% specify constraints for MV and MV Rate
mpcobj.MV(1).Min = -3 - FF_L;
mpcobj.MV(1).Max = 14 - FF_L;
mpcobj.MV(1).RateMin = -10*Ts;
mpcobj.MV(1).RateMax = 10*Ts;
mpcobj.MV(2).Min = -3 - FF_R;
mpcobj.MV(2).Max = 14 - FF_R;
mpcobj.MV(2).RateMin = -10*Ts;
mpcobj.MV(2).RateMax = 10*Ts;
mpcobj.MV(3).Min = -3 -rear_alfas;
mpcobj.MV(3).Max = 10 -rear_alfas;
mpcobj.MV(3).RateMin = -1*Ts;
mpcobj.MV(3).RateMax = 1*Ts;
mpcobj.MV(4).Min = 0-T;
mpcobj.MV(4).Max = 6000-T;
mpcobj.MV(4).RateMin = -2000*Ts;
mpcobj.MV(4).RateMax = 2000*Ts;
mpcobj.MV(5).Min = -15;
mpcobj.MV(5).Max = 15;
mpcobj.MV(5).RateMin = -2000*Ts;
mpcobj.MV(5).RateMax = 2000*Ts;
%% specify overall adjustment factor applied to weights
beta = 1.0833;
%% specify weights
mpcobj.Weights.MV = [0 0 0 0 0]*beta;
mpcobj.Weights.MVRate = [0.1 0.1 0.1 0.0001 0.1]/beta;
mpcobj.Weights.OV = [1 1 0.1 1 1]*beta;
mpcobj.Weights.ECR = 100000;
%% specify simulation options
options = mpcsimopt();
options.RefLookAhead = 'off';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';
save('mpc.mat','mpcobj')

minl = [-2 -0.3 -5 -15 -30];
maxl = [2 0.3 5 15 30];
N = 80000;
ref = minl + (maxl-minl) .* rand(N,1);
state_min = [-2 -1 -10 -3 -0.3 -1 -15 -30 -15];
state_max = [2 1 10 3 0.3 1 15 30 15];
state = state_min+(state_max-state_min).*rand(N,1);
minc = [-3-FF_L -3-FF_R -3-rear_alfas -T -15];
maxc = [14-FF_L 14-FF_R 10-rear_alfas 6000-T 15];
cont = minc+(maxc-minc).*rand(N,1);
res = zeros(5,N);
Boat_mpc = mpcstate(mpcobj);  
for j=1:N
    statempc = mpcstate(mpcobj,state(j,:),[],[],cont(j,:));
    res(:,j) = mpcmove(mpcobj,statempc,state(j,[1 5 4 9 8]),ref(j,:));
end
y = state(:,[1 5 4 9 8]);
xp = state;
u = res';
save('mpcdata.mat','y','u','ref','N','xp','cont');