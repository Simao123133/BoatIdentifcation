%Generates data from Alqueva tests
clear
load('dataTSB.mat');
%% Organize variables
state(1) = arrange(VESC1_RPM_MC_52_V1_RPM_02);
state(1).val = state(1).val/10;
state(2) = arrange(Foils_US_ang_39_right_angle_03);
state(3).val = 1.91*ones(length(state(1).time),1);
state(3).time = state(1).time;
state(4) = arrange(XCDI_Velocit_65_velX_00);
state(5) = arrange(XCDI_Velocit_65_velZ_02);
state(6) = arrange(XCDI_RateOfT_62_gyrY_01);
state(6).val = state(6).val*180/pi;
state(7) = arrange(XCDI_EulerAn_59_pitch_00);
state(8) = arrange(Foils_US_ang_39_dist_01);
state(9) = arrange(XCDI_Velocit_65_velY_01);
state(10) = arrange(XCDI_RateOfT_62_gyrX_00);
state(10).val = state(10).val*180/pi;
state(11) = arrange(XCDI_RateOfT_62_gyrZ_02);
state(11).val = state(11).val*180/pi;
state(12) = arrange(XCDI_EulerAn_59_roll_01);
state(13) = arrange(XCDI_EulerAn_59_yaw_02);
state(14) = arrange(Foils_US_ang_39_left_angle_02);

late_i = state(1).time(14631);
late_i_f = find(abs(state(14).time - late_i) < 0.04);

%% States start at same time and samples are organized
for i=1:1:length(state(14).time)-late_i_f
    for j=1:1:13
        indexes = find(abs(state(j).time -  state(14).time(i-1+late_i_f)) < 0.06);
        [~,index] = min(abs(state(j).time(indexes)- state(14).time(i-1+late_i_f)));
        state_c(i,j) = state(j).val(indexes(index));
    end
end

%% Get body frame velocities
%850 is when the boat begins accelarating
start = 850;
for i=start:length(state_c(:,13))
    state_c(i,14) = state(14).val(i+late_i_f-1);
    rot = rotdZ(state_c(i,13))*rotdY(state_c(i,7))*rotdX(state_c(i,12));
    V = rot*[state_c(i,4);state_c(i,9);state_c(i,5)];
    indexf=i-start+1;
    statef(indexf,4) = V(1);
    statef(indexf,9) = V(2);
    statef(indexf,5) = V(3);
    for j=1:14
        if j ~= 4 && j~= 5 && j~= 9
            statef(indexf,j) = state_c(i,j);
        end
    end
    
    %params = [statef(indexf,14) statef(indexf,2) statef(indexf,3) statef(indexf,1) statef(indexf,4) statef(indexf,9) statef(indexf,5) statef(indexf,10) statef(indexf,6) statef(indexf,11) statef(indexf,7) statef(indexf,12) statef(indexf,13) statef(indexf,8)];
    %[statef(indexf,15),fval] = fminsearch(@(x)FindRudderAngle(x,params),statef(indexf,11));
    %fvalv(indexf) = fval;
    statef(indexf,15) = statef(indexf,11)*2*7^2/statef(indexf,4)^2*statef(indexf,8)/65;
   
end

names = ["Motor",'Right_foil','Rear_foil','U','W','gyrY','Pitch','Height','V','gyrX','gyrZ','Roll','Yaw','Left_foil','Rudder'];
units = {'Rpm','deg','deg','m/s','m/s','deg/s','deg','m','m/s','deg/s','deg/s','deg','deg','deg','deg'};

delete raw_data.mat
for i=1:15
   s.(names(i)).unit = units{i};
   s.(names(i)).time = [0:0.1:length(statef(:,i))/10-0.1]';
   s.(names(i)).signals.values = statef(:,i);
end
%structvars(s);
Motor = s.Motor;               
Right_foil = s.Right_foil;     
Rear_foil = s.Rear_foil;       
U = s.U;                       
W = s.W;                                
Pitch = s.Pitch;               
Height = s.Height;                       
V = s.V;                                                 
Roll = s.Roll;                 
Yaw = s.Yaw;                   
Left_foil = s.Left_foil;       
Rudder = s.Rudder;             
gyrY = s.gyrY;                 
gyrX = s.gyrX;                 
gyrZ = s.gyrZ;  
save('raw_data.mat','Motor','Right_foil','Rear_foil','U','W','gyrY','Pitch','Height','V','gyrX','gyrZ','Roll','Yaw','Left_foil','Rudder');



function signal = arrange(data)
    signal.val = data.signals.values;
    signal.time = data.time;
end

