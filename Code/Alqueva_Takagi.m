%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file imports raw data measured in alqueva and derives one
% Takani-Sugeno Model.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%Load Data
load('data.mat')

%% %%%%%%%%%%%%%%%%%%%%%%%%%    cut Data    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cut data to have only point in stable flight

% Motor Angular Velocity
time_m = round(time_m,2);
start_m = find(time_m == 2486.96);
end_m = find(time_m == 4463.67);
time_m = time_m(start_m:end_m);
mspeed = mspeed(start_m:end_m);

% Foils Angles
time_foils = round(time_foils,2);
start_foils = find(time_foils == 2486.96);
end_foils = find(time_foils == 4463.67);
time_foils = time_foils(start_foils:end_foils);
foil_right = foil_right(start_foils:end_foils);
foil_left = foil_left(start_foils:end_foils);

% Velocity
time_vel = round(time_vel,2);
start_vel = find(time_vel == 2486.95);
end_vel = find(time_vel == 4463.66);
time_vel = time_vel(start_vel:end_vel);
vel = vel(start_vel:end_vel);

% Euler Angles
time_euler = round(time_euler,2);
start_euler = find(time_euler == 2486.95);
end_euler = find(time_euler == 4463.66);
time_euler = time_euler(start_euler:end_euler);
roll = roll(start_euler:end_euler);
pitch = pitch(start_euler:end_euler);
yaw = yaw(start_euler:end_euler);

% dPitch, dRoll
droll = droll(start_euler:end_euler);
dpitch = dpitch(start_euler:end_euler);




% Height
time_h = round(time_h,2);
start_h = find(time_h == 2486.96);
end_h = find(time_h == 4463.67);
time_h = time_h(start_h:end_h);
h_m = h_m(start_h:end_h);

%% %%%%%%%%%%%%%%%%%%%%% Resample data (50Hz) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
freq = 10; % [Hz]
delta_t = 1/freq; % [s]

% Motor Angular Velocity
time_m_50 = (time_m(1):delta_t:time_m(end))';
mspeed_50 = interp1(time_m,mspeed,time_m_50);

% Foils Angles
time_foils_50 = (time_foils(1):delta_t:time_foils(end))';
foil_left_50 = interp1(time_foils,foil_left,time_foils_50);
foil_right_50 = interp1(time_foils,foil_right,time_foils_50);

% Velocity
time_vel_50 = (time_vel(1):delta_t:time_vel(end))';
vel_50 = interp1(time_vel,vel,time_vel_50);

% Euler Angles
time_euler_50 = (time_euler(1):delta_t:time_euler(end))';
roll_50 = interp1(time_euler,roll,time_euler_50);
pitch_50 = interp1(time_euler,pitch,time_euler_50);
yaw_50 = interp1(time_euler,yaw,time_euler_50);

% dPitch, dRoll
droll_50 = interp1(time_euler,droll,time_euler_50);
% droll_50 = medfilt1(droll_50,10);
dpitch_50 = interp1(time_euler,dpitch,time_euler_50);
dpitch_50 = medfilt1(dpitch_50,10);

% Height
time_h_50 = (time_h(1):delta_t:time_h(end))';
h_m_50 = interp1(time_h,h_m,time_h_50);

% Back Foil angle (constant)
angle = 3;
foil_back_50 = angle*ones(length(time_foils_50),1);
% foil_back_50 = angle*(0.9+0.2*rand(length(time_foils_50),1));

% Organize data
input_data = [mspeed_50, foil_left_50, foil_right_50, foil_back_50];
% output_data = [vel_50, roll_50, pitch_50, yaw_50, h_m_50];
output_data = [vel_50, roll_50, pitch_50, droll_50, dpitch_50, h_m_50];

%% %%%%%%%%%%%%%%%%%%%%%%%%%% Plot Data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure,
plot(time_m_50,mspeed_50); hold on
plot(time_foils_50,foil_left_50)
plot(time_foils_50,foil_right_50)
plot(time_vel_50,vel_50)
plot(time_h_50,h_m_50)
plot(time_euler_50, yaw_50);
plot(time_euler_50, pitch_50);
plot(time_euler_50, dpitch_50);
plot(time_euler_50, droll_50); 
plot(time_euler_50, roll_50); hold off

%% %%%%%%%%%%%%%%%%%%%%%% Divide data set %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Normalize dat set
for i=1:4
    range = max(input_data(:,i)) - min(input_data(:,i));
    x =(input_data(:,i) - min(input_data(:,i))) / range;
    input_data(:,i) = 2*x-1; 
end

for i=1:6
    range = max(output_data(:,i)) - min(output_data(:,i));
    x=(output_data(:,i) - min(output_data(:,i))) / range;
    output_data(:,i)=2*x-1;    
end

inicial=4000;
n1=inicial+8000;
n2=n1+200;
% Train data
input_train = input_data(inicial:n1,:);
output_train = output_data(inicial:n1,:);
% Validation data
input_val = input_data(n1+1:n2,:);
output_val = output_data(n1+1:n2,:);


%% %%%%%%%%%%%%%%%%%%%% Fuzy Model Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Par.c = 6; % number of clusters
Par.m = 2; % fuzziness parameter
Par.tol = 0.01; % termination criterion
Par.ante = 2; % product-space MFs
Par.cons = 2; % Local LS

% MISO
% Par.Ny = [0]; % number of lagged outputs
% Par.Nu = [1 3 3 0]; % number of lagged inputs
% Par.Nd = [10 2 2 2]; % number of transport delays

% Old version (no derivatives)
% Par.Ny = [5 1 1 1 0
%           1 0 0 0 0
%           1 0 0 0 0
%           0 0 0 1 0
%           0 0 0 0 0]; % number of lagged outputs
% Par.Nu = [5 0 0 0
%           0 1 1 0
%           0 1 1 0
%           0 1 1 0
%           1 3 3 0]; % number of lagged inputs
% Par.Nd = [10 2 2 2
%           10 2 2 2
%           10 2 2 2 
%           10 2 2 2
%           10 2 2 2]; % number of transport delays


Par.Ny = eye(5,5);  % number of lagged outputs
Par.Nu = ones(6,4); % number of lagged inputs
Par.Nd = ones(6,4); % number of transport delays


Dat.Ts = delta_t;
Dat.U = input_train;
Dat.Y = output_train;

%Train Model
FM = fmclust(Dat,Par);


% % plotmfs(FM);
% % plotout(FM);
% % plot(ylm{1}(:,2));

%% Validate Model
[ym,VAF] = fmsim(input_val,output_val,FM); VAF










