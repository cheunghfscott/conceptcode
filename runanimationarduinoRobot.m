P1=[0 0];
axis([-100 150 -100 100])

 figure(3)
hold on 
plot(xtva,ytva,'g')
runnumber= size(xtva)

    t=1
while t < runnumber(2)
    axis([-100 150 -100 120])
  h = scatter(xtva(t),ytva(t));
   x1=subs(L1x,A1,Theta1(t));
   y1=subs(L1y,A1,Theta1(t));
   x2=subs(L2x,[A1,A2],[Theta1(t),Theta2(t)]);
   y2=subs(L2y,[A1,A2],[Theta1(t),Theta2(t)]);
   x3=subs(xe, [A1,A2,A3],[Theta1(t),Theta2(t),Theta3(t)]);
   y3=subs(ye, [A1,A2,A3],[Theta1(t),Theta2(t),Theta3(t)]);
   
   
   P2 = [ x1 y1];
   P3= [ x2 y2];
   P4=[x3 y3];
   
  % P1_circle = viscircles(P1,0.5);
   link1 = line([P1(1) P2(1)],[P1(2) P2(2)],'LineWidth',1);
   link2 = line([P2(1) P3(1)],[P2(2) P3(2)],'LineWidth',1);
   link3 = line([P3(1) P4(1)],[P3(2) P4(2)],'LineWidth',1);
   
   pause(0.005)
   
   delete(h);
   delete(link1)
   delete(link2)
   delete(link3)
   
    t*Tstep
    t=t+1;
end

hold off