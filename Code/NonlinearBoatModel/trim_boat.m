    %% Search for a specified operating point for the model - boat_model.

%ATTENTION BEFORE RUNNING
%open the 'boat_model' and set all 4 inputs as inputs with the manual
%switches

% too have free rear foil put -10 as input;
% too have free thrust put 0 as input;


function [] = trim_boat(initial_uspeed,initial_Z)


% wait bar
f = uifigure('Name','Wait','Position',[200 200 425 275]);
d = uiprogressdlg(f,'Title','Searching equilibrium point','Indeterminate','on');



%% Specify the model name
model = 'boat_model';

%set max and min values for trim
minAOA = -3; %min angle of attack
maxAOA = 15; %max angle of attack
minRpms_motor = 0;
maxRpms_motor = 6000; %max rpms
minGG = -15;
maxGG = 15;
minTheta = -5;
maxTheta = 5;
minU = 0;
maxU = 20;
minW = -2;
maxW = 2;
minZ = -0.8;
maxZ = 0;
minYAW = -50000;
maxYAW = 50000;
minV = -2;
maxV = 2;
Rpms_motor = 1000*initial_uspeed;

if initial_Z == -0.2
    foilborn = 0;
    angulo_frente = -3;
    angulo_tras = -3;
else
    foilborn = 1;
    angulo_frente = 7;
    angulo_tras = 7;
end
if initial_uspeed >= 7 && initial_Z < -0.3
    angulo_frente = 1;
    angulo_tras =1;
end
if initial_uspeed < 0.6
    Rpms_motor = 2300*initial_uspeed;
end

%% Set initial conditions
x0 = [initial_uspeed; 0; 0; 0;initial_Z; 0;0;0;0;0];
u0 = [angulo_frente;angulo_frente;angulo_tras;Rpms_motor;0];
y0 = [0 0 0 0 0 0 0 0];
% create constants for simulink model
assignin('base','initial_uspeed',x0(1));
assignin('base','initial_wspeed',x0(2));
assignin('base','initial_Q',x0(3));
assignin('base','initial_Pitch',x0(4));
assignin('base','initial_Z',x0(5));
assignin('base','initial_vspeed',x0(6));
assignin('base','initial_P',x0(7));
assignin('base','initial_R',x0(8));
assignin('base','initial_ROLL',x0(9));
assignin('base','initial_YAW',x0(10));

%% Set the constraints on the states in the model.
% - The defaults for all states are Known = false, SteadyState = true,
%   Min = -Inf, Max = Inf, dxMin = -Inf, and dxMax = Inf.


%% Create the operating point specification object.
opspec = operspec(model);

%% Set the constraints on the states in the model.
% - The defaults for all states are Known = false, SteadyState = true,
%   Min = -Inf, Max = Inf, dxMin = -Inf, and dxMax = Inf.

for i=1:length(opspec.States)
    opspec.States(i).x = x0(i);
end

for i=1:length(opspec.Outputs)
    opspec.Outputs(i).y = y0(i);
end


for i=1:(length(opspec.States))
    opspec.States(i).Known = true;
end
opspec.States(5).Known = foilborn;
opspec.States(6).Known = 1;
opspec.States(7).Known = 1;
opspec.States(8).Known = 1;
opspec.States(9).Known = 1;
opspec.States(10).Known = 1;
opspec.States(6).SteadyState = 1;
opspec.States(7).SteadyState = 1;
opspec.States(8).SteadyState = 0;
opspec.States(9).SteadyState = 1;
opspec.States(10).SteadyState = 1;
%% Set the constraints on the inputs in the model.
% - The defaults for all inputs are Known = false, Min = -Inf, and
% Max = Inf.

for i=1:length(opspec.inputs)
    opspec.Inputs(i).u = u0(i);
end
opspec.Inputs(5).known = 0;
%% set the max and mins

% Input (1) - boat_model/frontfoil_left
opspec.Inputs(1).Min = minAOA;
opspec.Inputs(1).Max = maxAOA;
% Input (2) - boat_model/frontfoil_right
opspec.Inputs(2).Min = minAOA;
opspec.Inputs(2).Max = maxAOA;
% Input (3) - boat_model/rear_foil
opspec.Inputs(3).Min = minAOA;
opspec.Inputs(3).Max = maxAOA;
% Input (4) - boat_model/thrust
opspec.Inputs(4).Min = minRpms_motor;
opspec.Inputs(4).Max = maxRpms_motor;
% Input (5) - boat_model/gg
opspec.Inputs(5).Min = minGG;
opspec.Inputs(5).Max = maxGG;

% States (3) - boat_model/theta
opspec.States(4).Min = minTheta;
opspec.States(4).Max = maxTheta;
% States (5) - boat_model/uspeed
opspec.States(1).Min = minU;
opspec.States(1).Max = maxU;
% States (7) - boat_model/wspeed
opspec.States(2).Min = minW;
opspec.States(2).Max = maxW;
% States (10) - boat_model/z
opspec.States(5).Min = minZ;
opspec.States(5).Max = maxZ;
% States (8) - boat_model/yaw
opspec.States(10).Min = minYAW;
opspec.States(10).Max = maxYAW;
% States (6) - boat_model/v
opspec.States(6).Min = minV;
opspec.States(6).Max = maxV;

%% Set the constraints on the outputs in the model.
% - The defaults for all outputs are Known = false, Min = -Inf, and
% Max = Inf.

%% Create the search options
opt = findopOptions;

opt.DisplayReport = 'iter';
opt.OptimizerType = 'graddescent-elim';


%% Perform the operating point search
 
    [op,opreport] = findop(model,opspec,opt)
    
    FF_L = op.inputs(1).u;
    FF_R = op.inputs(2).u;
    rear_alfas = op.inputs(3).u;
    T = op.inputs(4).u;
    gg = op.inputs(5).u;
    initial_uspeed = op.states(1).x;
    initial_wspeed = op.states(2).x;
    initial_Q = op.states(3).x;
    initial_Pitch = op.states(4).x;
    initial_Z = op.states(5).x;
    initial_vspeed = op.states(6).x;
    initial_P = op.states(7).x;
    initial_R = op.states(8).x;
    initial_ROLL = op.states(9).x;
    initial_YAW = op.states(10).x;
    
    save('trim_op_fixed_v','op','FF_L','FF_R','rear_alfas','T','gg','initial_uspeed','initial_wspeed','initial_Q','initial_Pitch','initial_Z','initial_vspeed','initial_P','initial_R','initial_ROLL','initial_YAW')
    
close(f)

end

