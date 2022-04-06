%clear;clc;
prompt = {'Enter the speed at which to start:'};
answer = inputdlg(prompt);
answer = str2double(answer);
save('answer','answer');
trim_boat(7,-0.4);
linearize_boat;
lqi;
load('answer', 'answer')
trim_boat(answer,-0.4); 
load('trim_op_fixed_v')
load('controller')
load('lookuptable');
Kir = [0.1 0.8 -0.5]; %%better gains found experimentally
save('BoatMatrices','A','B');



