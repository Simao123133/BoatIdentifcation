figure(1)
states = ["u",'w','dPitch','pitch','z','v','p','dYaw','roll'];
ylabels =["m/s",'m/s','deg/s','deg','m','m/s','deg/s','deg/s','deg'];
for i=1:9
   subplot(3,3,i)
   if i == 1
       plot(out.states.time,out.states.Data(:,i),out.ref.time,out.ref.Data(:,1)); 
       legend('NN','Ref')
   elseif i==5
       plot(out.states.time,out.states.Data(:,i),out.ref.time,out.ref.Data(:,2)); 
       legend('NN','Ref')
   elseif i==9
       plot(out.states.time,out.states.Data(:,i),out.ref.time,out.ref.Data(:,3));
       legend('NN','Ref')
   elseif i==8
       plot(out.states.time,out.states.Data(:,i),out.ref.time,out.ref.Data(:,3));
       legend('NN','Ref')
   else
       plot(out.states.time,out.states.Data(:,i))
   end
   xlabel('Time [s]')
   ylabel(ylabels(i))
   title(states(i));
end

states2 = ["L_F",'R_F','Rudder','Motor','Rudder'];
ylabels2 =["[deg]",'[deg]','[deg]','[Rpms]','[deg/s]'];
figure(2)
for i=1:4
   subplot(2,3,i)
   plot(out.inputsNN.time,out.inputsNN.Data(:,i)); 
   xlabel('Time [s]')
   ylabel(ylabels2(i))
   title(states2(i));
end