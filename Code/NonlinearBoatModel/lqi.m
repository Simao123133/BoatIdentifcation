%%%%%%%%%%%%%%%% Integral action state is added for reference tracking: variable x_i
% x_i_dot = C*x - r

load('linear_model.mat')
load('QeR_longitudinal')
load('QeR_lateral_com_v')
% New A, B, C matrices

Aih=[Ah   , zeros(size(Ah,1),1);
    -Ch   ,        zeros(1,1)      ];

Bih=[ Bh; 
     0];
     
Cih=[Ch, zeros(1,1)];

Dih=Dh;

%%%%%%%%%%%%%%%%% New LQR matrices

% normalize and set Q matrix weights then set R for controller and calculate poles
theta_range = 5;
thetadot_range = 20; 
u_range = 10;
w_range = 4;
z_range = 0.5;

foil_range = 4;

weigths = [1 1 1 1 1];

eta = [weigths(1)/u_range^2 weigths(2)/w_range^2 weigths(3)/thetadot_range^2 ...
       weigths(4)/theta_range^2 weigths(5)/z_range^2 ]; 


Qih = Q_1foil;
Rih = R_1foil;
[Kih,~,~]=lqr(Aih,Bih,Qih,Rih);
Br_h = [zeros(size(Ah,1),1);1];

model_h = ss(Aih-Bih*Kih,Br_h,Cih,Dih);
%model_h = ss([Ah-Bh*Kx -Bh*Ki;-Ch zeros(3,3)],Br_h,Cih,Dih);
%[num,den] = ss2tf(Aih-Bih*Kih,Br_h,Cih,Dih,1);
%funcao = tf(num(1,:),den);
%margin(funcao)
%S = allmargin(model_h);
%damp(model_h);
%step(model_h);
% New A, B, C matrices

Ail=[Al   , zeros(size(Al,1),1);
    Cl   ,        0      ];

Bil=[ Bl(:,1); 
     0];
     
Cil=[Cl, zeros(size(Cl,1),1)];

Dil=0;


%%%%%%%%%%%%%%%%% New LQR matrices

Qil = Q_lateral_com_v;
Ril= R_lateral_com_v;
[Kir,~,~]=lqr(Ail,Bil,Qil,Ril);

Br_l = [zeros(size(Al,1),1);1];
model_l = ss(Ail-Bil*Kir,Br_l,Cil,Dil);
%damp(model_l);
    

save('controller','Kih','Kir')


 
