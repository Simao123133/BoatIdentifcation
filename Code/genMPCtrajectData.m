% Generates MPC trajectories data to train the NN
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
Boat = c2d(cont_Boat,0.02);
%% create MPC controller object with sample time
mpcobj = mpc(Boat);
%% specify prediction horizon
mpcobj.PredictionHorizon = 15;
%% specify control horizon
mpcobj.ControlHorizon = 3;
%% specify nominal values for inputs and outputs
mpcobj.Model.Nominal.U = [0;0;0;0;0];
mpcobj.Model.Nominal.Y = [0;0;0;0;0];
%% specify constraints for MV and MV Rate
mpcobj.MV(1).Min = -3 - FF_L;
mpcobj.MV(1).Max = 14 - FF_L;
mpcobj.MV(1).RateMin = -10;
mpcobj.MV(1).RateMax = 10;
mpcobj.MV(2).Min = -3 - FF_R;
mpcobj.MV(2).Max = 14 - FF_R;
mpcobj.MV(2).RateMin = -10;
mpcobj.MV(2).RateMax = 10;
mpcobj.MV(3).Min = -3 -rear_alfas;
mpcobj.MV(3).Max = 10 -rear_alfas;
mpcobj.MV(3).RateMin = -1;
mpcobj.MV(3).RateMax = 1;
mpcobj.MV(4).Min = 0-T;
mpcobj.MV(4).Max = 6000-T;
mpcobj.MV(4).RateMin = -2000;
mpcobj.MV(4).RateMax = 2000;
mpcobj.MV(5).Min = -15;
mpcobj.MV(5).Max = 15;
mpcobj.MV(5).RateMin = -2000;
mpcobj.MV(5).RateMax = 2000;
%% specify overall adjustment factor applied to weights
beta = 1.0833;
%% specify weights
mpcobj.Weights.MV = [0 0 0 0 0]*beta;
mpcobj.Weights.MVRate = [0.1 0.1 0.1 0.0001 0.0001]/beta;
mpcobj.Weights.OV = [3 3 0.3 10 10]*beta;
mpcobj.Weights.ECR = 100000;
%% specify simulation options
options = mpcsimopt();
options.RefLookAhead = 'off';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';
minl = [-2 -0.3 -5 -15 -30];
maxl = [2 0.3 5 15 30];
N = 1000;
r = minl + (maxl-minl) .* rand(N,1);
%y = zeros(N,200,5);
u = zeros(N,200,5);
xp = zeros(N,200,9);
y = zeros(N,200,5);
Boat_mpc = mpcstate(mpcobj);
%% run simulation
for i=1:1:N
    mpcobj_RefSignal = r(i,:);
    [y(i,:,:),~,u(i,:,:), xp(i,:,:)] = sim(mpcobj, 200, mpcobj_RefSignal, [], options);
end
save('mpcdata.mat','y','u','r','N');

r = [-1 0 0 0 0];
tic
MPCopt = mpcmoveopt;
MPCopt.MVRate
MPCopt.MVMax = 2;
u1 = mpcmove(mpcobj,Boat_mpc,reshape(y(i,1,:), [1 5]),r(i,:));
toc