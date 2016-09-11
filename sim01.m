%----------------------------------------------------
% Simulation Single gimbaled VSCMG
%              by Hyungjoo Yoon (11/13/2000)
%  (Acceleration Steering law)
%----------------------------------------------------

format short; clear; close all;

% --- Initialize ---------------------------------
initialize;

x0=[h;sigma;gamma;gamma_dot;Omega];

t0=0;tf=100;

global flag;
flag=0;
%options = odeset('RelTol',1e-5,'AbsTol',1e-5);
%[t,x] = ode23('dynamics_vscmg',[t0,tf],x0,options);
[t,x] = ode23('dynamics_vscmg',[t0 tf],x0);

sz=size(t);sz=sz(1);
w=zeros(sz,3);
V1=zeros(sz,1);
global k0;
for i=1:sz   
   As=As0*diag(cos(x(i,7:9)))+At0*diag(sin(x(i,7:9)));
   At=At0*diag(cos(x(i,7:9)))-As0*diag(sin(x(i,7:9)));
   Ag=Ag0;
   J=I_B + As*Ics*As' + At*Ict*At' + Ag*Icg*Ag';
   w(i,1:3)=(inv(J)*(x(i,1:3)'-Ag*Icg*x(i,10:12)'-As*Iws*x(i,13:15)'))';
   V1(i,1)=0.5*w(i,1:3)*J*w(i,1:3)'+2*k0*log(1+x(i,4:6)*x(i,4:6)');
end

figure;
subplot(5,1,1);plot(t,x(:,1:3));title('h');
subplot(5,1,2);plot(t,x(:,4:6));title('sigma');
%subplot(3,1,3);
%plot(t,x(:,1).^2+x(:,2).^2+x(:,3).^2);
subplot(5,1,3);plot(t,w(:,1:3));title('omega');

%figure;
subplot(5,1,4);plot(t,x(:,7:9)*180/(2*pi));title('gamma');
subplot(5,1,5);plot(t,x(:,13:15));title('Omega');

figure;plot(t,V1);


