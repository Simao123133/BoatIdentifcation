% Este script implementa o PSO MPC com base na multistep feedforward net
% Define the details of the table design problem
nVar = 40;
for i = 1:10
ub(4*i-3:4*i) = [11 11 8000 1];
lb(4*i-3:4*i) = [6 6 6000 -1];
end
% Define the PSO's paramters
noP = 1000;
maxIter =20;
wMax = 0.9;
wMin = 0.4;
c1 = 2;
c2 = 2;
vMax = (ub - lb) .* 0.2;
vMin  = -vMax;
mdl = 'boat_model_and_controller';
% The PSO algorithm
rand('state',sum(100.*clock));
% Initialize the particles

Swarm_Particles_X = (ub-lb) .* rand(noP,nVar) + lb;
Swarm_Particles_V = zeros(noP, nVar);
Swarm_Particles_PBEST_X = zeros(noP,nVar);
Swarm_Particles_PBEST_O = inf(1,noP);

Swarm_GBEST_X = zeros(1,nVar);
Swarm_GBEST_O = inf;
initial_state = [7.1 0 0 2.5 65 0 0 0 0];
ref = zeros(90,1);
indexes = 0;
zindexes = 0;
for i=1:10
ref(9*i-8) = 7.1;
ref(9*i-7) = 0;
ref(9*i-6) = 0;
ref(9*i-5) = 2.5;
ref(9*i-4) = 6.5;
ref(9*i-3) = 0;
ref(9*i-2) = 0;
ref(9*i-1) = 0;
ref(9*i) = 0;
indexes = [indexes 9*i-8 9*i-4 9*i-1 9*i];
zindexes = [zindexes 9*i-4];
end
indxes = indexes(2:end);
zindxes = zindexes(2:end);
Swarm_Particles_O = zeros(1,noP);
t = 1;
% Main loop
inputs = zeros(1,49);
inputs(1:9) = initial_state;
while t<=maxIter
    % Calcualte the objective value
    for k=1:noP
        inputs(10:49) = Swarm_Particles_X(k,:);
        %for i = 1:9
        %inputs(4*i+10:4*i+13) = Swarm_Particles_X(k,5:8);
        %end
        pred = net(inputs');
        pred(zindxes) = pred(zindxes)/10;
        Swarm_Particles_O(k) = sum((pred(indxes)-ref(indxes)).^2);
    end
    index = find(Swarm_Particles_O - Swarm_Particles_PBEST_O < 0);
    Swarm_Particles_PBEST_X(index,:) = Swarm_Particles_X(index,:);
    Swarm_Particles_PBEST_O(index) = Swarm_Particles_O(index);
    
    [Omin,Pmin] = min(Swarm_Particles_O);
    aux = Swarm_GBEST_O;
    if Omin < Swarm_GBEST_O
        Swarm_GBEST_X = Swarm_Particles_X(Pmin,:);
        Swarm_GBEST_O = Omin;
    end
    % Update the X and V vectors
    w = wMax - t .* ((wMax - wMin) / maxIter);
    
    
    Swarm_Particles_V = w*Swarm_Particles_V + c1.*rand(noP,nVar).*(Swarm_Particles_PBEST_X - Swarm_Particles_X) ...
        + c2 .* rand(noP,nVar) .* (Swarm_GBEST_X - Swarm_Particles_X);
    
    
    % Check velocities
    
    for k=1:nVar
        Swarm_Particles_V(Swarm_Particles_V(:,k) > vMax(k),k) = vMax(k);
        Swarm_Particles_V(Swarm_Particles_V(:,k) < vMin(k),k) = vMin(k);
        
    end
    
    Swarm_Particles_X = Swarm_Particles_X + Swarm_Particles_V;
    
    for k=1:nVar
        
        Swarm_Particles_X(Swarm_Particles_X(:,k) > ub(k),k) = ub(k);
        Swarm_Particles_X(Swarm_Particles_X(:,k) < lb(k),k) = lb(k);
    end
    
    save('foiabaixo.mat','Swarm_Particles_V','Swarm_Particles_X','Swarm_GBEST_X','Swarm_GBEST_O','Swarm_Particles_PBEST_X','Swarm_Particles_PBEST_O','t')
    outmsg = ['Iteration# ', num2str(t) , ' Swarm_GBEST_O = ' , num2str(Swarm_GBEST_O)];
    disp(outmsg);
    t = t+1;
end
inputs(10:49) = Swarm_GBEST_X;
y = net(inputs');

titles = ["u",'w','dPitch','pitch','z','v','dRoll','dYaw','roll'];
labels =["m/s",'m/s','deg/s','deg','m','m/s','deg/s','deg/s','deg'];
figure(1)
results = y';
for h=1:10
    resy(h,1) = results(9*h-8);
    resy(h,2) = results(9*h-7);
    resy(h,3) = results(9*h-6);
    resy(h,4) = results(9*h-5);
    resy(h,5) = results(9*h-4);
    resy(h,6) = results(9*h-3);
    resy(h,7) = results(9*h-2);
    resy(h,8) = results(9*h-1);
    resy(h,9) = results(9*h);
    resu(h,1) = inputs(4*h+6);
    resu(h,2) = inputs(4*h+7);
    resu(h,3) = inputs(4*h+8);
    resu(h,4) = inputs(4*h+9);
end

for i=1:9
subplot(3,3,i);
plot(0:0.1:length(resy)/10-0.1, resy(:,i));
title(titles(i))
xlabel('Time [s]')
ylabel(labels(i))
end

titles2 = ["LF",'RF','Motor','Rudder'];
labels2 =["deg",'deg','Rpms','deg','m'];
figure(2)

for i=1:4
subplot(2,2,i);
plot(0:0.1:0.9, resu(:,i));
title(titles2(i))
xlabel('Time [s]')
ylabel(labels2(i))
end