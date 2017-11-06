clear
l1=50;
l2=45;
l3=35;
syms A1 A2 A3 t
L1x= l1*cos(A1);
L1y= l1*sin(A1);
L2x=l1*cos(A1)+l2*cos(A1+A2);
L2y=l1*sin(A1)+l2*sin(A1+A2);

xe=l1*cos(A1)+l2*cos(A1+A2)+l3*cos(A1+A2+A3);
ye=l1*sin(A1)+l2*sin(A1+A2)+l3*sin(A1+A2+A3);

runtill=20;                             %Run till
Tstep=0.05 ;                            %time step
plottime=5.3;                             %Plot at specific time for links and target
G1=[10 0;0 4];
G2=100;

xedot= [-l1*sin(A1)-l2*sin(A1+A2)-l3*sin(A1+A2+A3) -l2*sin(A1+A2)-l3*sin(A1+A2+A3) -l3*sin(A1+A2+A3)];
yedot= [ l1*cos(A1)+l2*cos(A1+A2)+l3*cos(A1+A2+A3) l2*cos(A1+A2)+l3*cos(A1+A2+A3) l3*cos(A1+A2+A3)];
Aedot = [ 1 1 1];

xev=zeros(1,runtill/Tstep);
yev=zeros(1,runtill/Tstep);

Theta1=zeros(1,runtill/Tstep);
Theta2=zeros(1,runtill/Tstep);
Theta3=zeros(1,runtill/Tstep);
timecounter=zeros(1,runtill/Tstep);

Theta1(1)=(3/4)*pi;                 
Theta2(1)=-0.125*pi;
Theta3(1)=-0.125*pi;


xtva=zeros(1,runtill/Tstep);          %Define trajectoary variables
ytva=zeros(1,runtill/Tstep);
J1s=zeros(1,runtill/Tstep);

xt1=-0.2*t^3+3*t^2;             %trajectary equations
xt1= 5*t;
xt2=0.3*t^3-12.5*t^2+160*t-550;
yt1=70;
yt2=0.1786*t^3-8.0357*t^2+107.1429*t-378.4286;

rd1=[xt1,yt1];          %trajectary vectors
rd1dot=[diff(xt1,t);diff(yt1,t)];
rd2dot=[diff(xt2,t);diff(yt2,t)];
rt=0;

J1= [xedot;yedot];
J2=Aedot;
J2p=pinv(J2);
%J2h=J2p*(eye(3)-pinv(J1t)*J1t);
time=0;
xtva(1)= subs(xt1,t,time);  % trajectary values
ytva(1)= subs(yt1,t,time);
xev(1)=subs(xe,[A1,A2,A3],[Theta1(1),Theta2(1),Theta3(1)]);
yev(1)=subs(ye,[A1,A2,A3],[Theta1(1),Theta2(1),Theta3(1)]);

time=Tstep;
n=1;

while time<runtill
    fprintf('time is:')
    time
    %tic
        J1t= double(subs(J1,[A1,A2,A3],[Theta1(n),Theta2(n),Theta3(n)]));
        pos=double([subs(xe,[A1,A2,A3],[Theta1(n),Theta2(n),Theta3(n)]);subs(ye,[A1,A2,A3],[Theta1(n),Theta2(n),Theta3(n)])]);
        ori=Theta1(n)+Theta2(n)+Theta3(n);  %orientation
        J2h=J2*(eye(3)-pinv(J1t)*J1t);
    %fprintf('1st') 
    %toc
        if time<= 10
        %tic
        xtv= subs(xt1,t,time);
        ytv= subs(yt1,t,time);
        h1= subs(rd1dot,t,time)+G1*([xtv;ytv]-pos );
        %fprintf('2nd') 
        %toc
        xtva(n+1)= subs(xt1,t,time);
        ytva(n+1)= subs(yt1,t,time);
    elseif time > 10
        
         xtva(n+1)= subs(xt2,t,time);
         ytva(n+1)= subs(yt2,t,time);
         xtv= subs(xt2,t,time);
         ytv= subs(yt2,t,time);
         h1= double(subs(rd2dot,t,time)+G1*([xtv;ytv]-pos ));
        
        end
    %tic
     h2=G2*(pi/2-ori);
    thetadot= pinv(J1t)*h1+(eye(3)-pinv(J1t)*J1t)*J2p*h2;
        
    Newtheta= [Theta1(n);Theta2(n);Theta3(n)]+thetadot*Tstep;
    %fprintf('3rd') 
    
    %toc
    
    %tic
      timecounter(n+1)=time;
      xev(n+1)=pos(1);
      yev(n+1)=pos(2);
      
      Theta1(n+1)=Newtheta(1);
      Theta2(n+1)=Newtheta(2);
      Theta3(n+1)=Newtheta(3);
      
      
      
      
      time=time+Tstep;
    J1s(n)=sqrt(det(J1t'*J1t)); 
      n=n+1;
     % fprintf('4th') 
    
      %  toc
end

 %time = 2
 
 
 xtv= subs(xt1,t,plottime);
 ytv= subs(yt1,t,plottime);
 
 plottime
 
 stepplot=int16(plottime/Tstep)+1;
Xerror = abs(xtva)-abs(xev);
Yerror= abs(ytva)-abs(yev);
AllTheta=Theta1+Theta2+Theta3;
Thetaerror= (AllTheta)-pi/2;

tplot=[0:Tstep:runtill-Tstep];

figure(1)
subplot(3,1,1);
plot(tplot,Xerror)
title('Subplot 1: x-error')
subplot(3,1,2);
plot(tplot,Yerror)
title('Subplot 2: y-error')


subplot(3,1,3)
plot(tplot,Thetaerror)
title('Subplot 3: Theta-error')

runnumber= size(xtva);
%calculating average error
XaError= double(sum(Xerror)/runnumber(2));
YaError= double(sum(Yerror)/runnumber(2));
ThetaAerror=  double(sum(Thetaerror)/runnumber(2));
ErrorA=[XaError;YaError;ThetaAerror]
%calculating error in t1
Xt1Error= double(sum(Xerror(1:runnumber(2)/2)/runnumber(2)/2));
Yt1Error= double(sum(Yerror(1:runnumber(2)/2)/runnumber(2)/2));
Thetat1error=  double(sum(Thetaerror(1:runnumber(2)/2))/runnumber(2)/2);
Errort1=[Xt1Error;Yt1Error;Thetat1error]
%calculating error in t2
Xt2Error= double(sum(Xerror(runnumber(2)/2:runnumber(2))/runnumber(2)/2));
Yt2Error= double(sum(Yerror(runnumber(2)/2:runnumber(2))/runnumber(2)/2));
Thetat2error=  double(sum(Thetaerror(runnumber(2)/2:runnumber(2))/runnumber(2)/2));
Errort2=[Xt2Error;Yt2Error;Thetat2error]

