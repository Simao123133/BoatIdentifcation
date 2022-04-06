%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file initializes the fuzzy systems used for PID controller in
% simulink model. Run this File before running the Simulink Model.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Fuzzy model for velocity control
fis_u = sugfis;

% Input: velocity error
fis_u = addInput(fis_u,[-6 6],'Name','delta_vel'); 
fis_u = addMF(fis_u,'delta_vel','trimf',[-7 -6 0],'Name','N');
fis_u = addMF(fis_u,'delta_vel','trimf',[-6 0 6],'Name','Z');
fis_u = addMF(fis_u,'delta_vel','trimf',[0 6 7],'Name','P');

% figure,
% plotmf(fis_u,'input',1)
% title('Input 1')

% Output: Motor rpms
fis_u = addOutput(fis_u,[-4000 4000],'Name','RPM');
fis_u = addMF(fis_u,'RPM','constant',4000,'Name','I');
fis_u = addMF(fis_u,'RPM','constant',0,'Name','Z');
fis_u = addMF(fis_u,'RPM','constant',-4000,'Name','D');

% Rules
rules = [...
    "delta_vel==N => RPM=D"; ...
    "delta_vel==Z => RPM=Z"; ...
    "delta_vel==P => RPM=I"; ...
    ];
fis_u = addRule(fis_u,rules);

% figure,
% gensurf(fis_u)
% title('Control surface of type-1 FIS')
%% Fuzzy PID controller
fis1 = sugfis;
fis1 = addInput(fis1,[-1 1],'Name','E');
fis1 = addInput(fis1,[-1 1],'Name','delE');

fis1 = addMF(fis1,'E','trimf',[-2 -1 0],'Name','N');
fis1 = addMF(fis1,'E','trimf',[-1 0 1],'Name','Z');
fis1 = addMF(fis1,'E','trimf',[0 1 2],'Name','P');
fis1 = addMF(fis1,'delE','trimf',[-2 -1 0],'Name','N');
fis1 = addMF(fis1,'delE','trimf',[-1 0 1],'Name','Z');
fis1 = addMF(fis1,'delE','trimf',[0 1 2],'Name','P');


fis1 = addOutput(fis1,[-1 1],'Name','U');

fis1 = addMF(fis1,'U','constant',-1,'Name','NB');
fis1 = addMF(fis1,'U','constant',-0.5,'Name','NM');
fis1 = addMF(fis1,'U','constant',0,'Name','Z');
fis1 = addMF(fis1,'U','constant',0.5,'Name','PM');
fis1 = addMF(fis1,'U','constant',1,'Name','PB');


rules = [...
    "E==N & delE==N => U=NB"; ...
    "E==Z & delE==N => U=NM"; ...
    "E==P & delE==N => U=Z"; ...
    "E==N & delE==Z => U=NM"; ...
    "E==Z & delE==Z => U=Z"; ...
    "E==P & delE==Z => U=PM"; ...
    "E==N & delE==P => U=Z"; ...
    "E==Z & delE==P => U=PM"; ...
    "E==P & delE==P => U=PB" ...
    ];
fis1 = addRule(fis1,rules);


%% Fuzzy PID controller U

fis2 = sugfis;
fis2 = addInput(fis2,[-6 6],'Name','E');
fis2 = addInput(fis2,[-6 6],'Name','delE');

fis2 = addMF(fis2,'E','trimf',[-7 -6 0],'Name','N');
fis2 = addMF(fis2,'E','trimf',[-6 0 6],'Name','Z');
fis2 = addMF(fis2,'E','trimf',[0 6 7],'Name','P');
fis2 = addMF(fis2,'delE','trimf',[-7 -6 0],'Name','N');
fis2 = addMF(fis2,'delE','trimf',[-6 0 6],'Name','Z');
fis2 = addMF(fis2,'delE','trimf',[0 6 7],'Name','P');


fis2 = addOutput(fis2,[-4000 4000],'Name','U');

fis2 = addMF(fis2,'U','constant',-4000,'Name','NB');
fis2 = addMF(fis2,'U','constant',-2000,'Name','NM');
fis2 = addMF(fis2,'U','constant',0,'Name','Z');
fis2 = addMF(fis2,'U','constant',2000,'Name','PM');
fis2 = addMF(fis2,'U','constant',4000,'Name','PB');


rules = [...
    "E==N & delE==N => U=NB"; ...
    "E==Z & delE==N => U=NM"; ...
    "E==P & delE==N => U=Z"; ...
    "E==N & delE==Z => U=NM"; ...
    "E==Z & delE==Z => U=Z"; ...
    "E==P & delE==Z => U=PM"; ...
    "E==N & delE==P => U=Z"; ...
    "E==Z & delE==P => U=PM"; ...
    "E==P & delE==P => U=PB" ...
    ];
fis2 = addRule(fis2,rules);


%% Fuzzy PD controller Pitch

fis3 = sugfis;
fis3 = addInput(fis3,[-10 10],'Name','E');
fis3 = addInput(fis3,[-10 10],'Name','delE');

% Input: erro pitch e respetiva derivada
fis3 = addMF(fis3,'E','trimf',[-11 -10 0],'Name','N');
fis3 = addMF(fis3,'E','trimf',[-10 0 10],'Name','Z');
fis3 = addMF(fis3,'E','trimf',[0 10 11],'Name','P');
fis3 = addMF(fis3,'delE','trimf',[-11 -10 0],'Name','N');
fis3 = addMF(fis3,'delE','trimf',[-10 0 10],'Name','Z');
fis3 = addMF(fis3,'delE','trimf',[0 10 11],'Name','P');

% Output: Back Foil angle
fis3 = addOutput(fis3,[-3 10],'Name','U');

fis3 = addMF(fis3,'U','constant',-3,'Name','NB');
fis3 = addMF(fis3,'U','constant',-1.5,'Name','NM');
fis3 = addMF(fis3,'U','constant',0,'Name','Z');
fis3 = addMF(fis3,'U','constant',5,'Name','PM');
fis3 = addMF(fis3,'U','constant',10,'Name','PB');


rules = [...
    "E==N & delE==N => U=PB"; ...
    "E==Z & delE==N => U=PM"; ...
    "E==P & delE==N => U=Z"; ...
    "E==N & delE==Z => U=PM"; ...
    "E==Z & delE==Z => U=Z"; ...
    "E==P & delE==Z => U=NM"; ...
    "E==N & delE==P => U=Z"; ...
    "E==Z & delE==P => U=NM"; ...
    "E==P & delE==P => U=NB" ...
    ];
fis3 = addRule(fis3,rules);

%% Fuzzy PD controller Roll

fis4 = sugfis;
fis4 = addInput(fis4,[-20 20],'Name','E');
fis4 = addInput(fis4,[-40 40],'Name','delE');

% Input: erro pitch e respetiva derivada
fis4 = addMF(fis4,'E','trimf',[-21 -20 0],'Name','N');
fis4 = addMF(fis4,'E','trimf',[-20 0 20],'Name','Z');
fis4 = addMF(fis4,'E','trimf',[0 20 21],'Name','P');
fis4 = addMF(fis4,'delE','trimf',[-41 -40 0],'Name','N');
fis4 = addMF(fis4,'delE','trimf',[-40 0 40],'Name','Z');
fis4 = addMF(fis4,'delE','trimf',[0 40 41],'Name','P');

% Output: Back Foil angle
fis4 = addOutput(fis4,[-3 14],'Name','U');

fis4 = addMF(fis4,'U','constant',-3,'Name','NB');
fis4 = addMF(fis4,'U','constant',-1.5,'Name','NM');
fis4 = addMF(fis4,'U','constant',0,'Name','Z');
fis4 = addMF(fis4,'U','constant',7,'Name','PM');
fis4 = addMF(fis4,'U','constant',14,'Name','PB');


rules = [...
    "E==N & delE==N => U=NB"; ...
    "E==Z & delE==N => U=NM"; ...
    "E==P & delE==N => U=Z"; ...
    "E==N & delE==Z => U=NM"; ...
    "E==Z & delE==Z => U=Z"; ...
    "E==P & delE==Z => U=PM"; ...
    "E==N & delE==P => U=Z"; ...
    "E==Z & delE==P => U=PM"; ...
    "E==P & delE==P => U=PB" ...
    ];
fis4 = addRule(fis4,rules);

%% Fuzzy PD controller Altura

fis5 = sugfis;
fis5 = addInput(fis5,[-1 1],'Name','E');
fis5 = addInput(fis5,[-0.5 0.5],'Name','delE');

% Input: erro altura e respetiva derivada
fis5 = addMF(fis5,'E','trimf',[-2 -1 0],'Name','N');
fis5 = addMF(fis5,'E','trimf',[-1 0 1],'Name','Z');
fis5 = addMF(fis5,'E','trimf',[0 1 2],'Name','P');
fis5 = addMF(fis5,'delE','trimf',[-0.6 -0.5 0],'Name','N');
fis5 = addMF(fis5,'delE','trimf',[-0.5 0 0.5],'Name','Z');
fis5 = addMF(fis5,'delE','trimf',[0 0.5 0.6],'Name','P');

% Output: comum angle
fis5 = addOutput(fis5,[-3 14],'Name','U');

fis5 = addMF(fis5,'U','constant',-3,'Name','NB');
fis5 = addMF(fis5,'U','constant',-1.5,'Name','NM');
fis5 = addMF(fis5,'U','constant',0,'Name','Z');
fis5 = addMF(fis5,'U','constant',7,'Name','PM');
fis5 = addMF(fis5,'U','constant',14,'Name','PB');


rules = [...
    "E==N & delE==N => U=NB"; ...
    "E==Z & delE==N => U=NM"; ...
    "E==P & delE==N => U=Z"; ...
    "E==N & delE==Z => U=NM"; ...
    "E==Z & delE==Z => U=Z"; ...
    "E==P & delE==Z => U=PM"; ...
    "E==N & delE==P => U=Z"; ...
    "E==Z & delE==P => U=PM"; ...
    "E==P & delE==P => U=PB" ...
    ];
fis5 = addRule(fis5,rules);
