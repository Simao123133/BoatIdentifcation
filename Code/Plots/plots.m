figure(1)
states = ["u",'z','roll','roll','dYaw'];
ylabels =["m/s",'m','deg','deg','deg/s'];
is = [7 -0.4 0 0 0];
for i=1:3
   subplot(1,3,i)
   plot(1,1,1,1);
   xlabel('Time [s]')
   ylabel(ylabels(i))
   legend('LQIs & PID','Ref')
   title(states(i));
end

states2 = ["L_F",'R_F','Motor'];
ylabels2 =["[deg]",'[deg]','[Rpms]'];
figure(2)
for i=1:3
   subplot(2,3,i)
   plot(out.inputsMPC.time,out.inputsMPC.Data(:,i)); 
   xlabel('Time [s]')
   ylabel(ylabels2(i))
   title(states2(i));
end

figure(4)
for i=1:5
   subplot(2,3,i)
   plot(out.statesNN.time,out.statesNN.Data(:,i),out.statesNN.time,ones(length(out.statesNN.time))*out.refMPC.Data(:,i)); 
   xlabel('Time [s]')
   ylabel(ylabels(i))
   title(states(i));
end

figure(3)
for i=1:5
   subplot(2,3,i)
   plot(out.inputsNN.time,out.inputsNN.Data(:,i)); 
   xlabel('Time [s]')
   ylabel(ylabels2(i))
   title(states2(i));
end