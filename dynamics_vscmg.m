function xd=dynamics_vscmg(t,x)

global As0 At0 Ag0;
global I_B Iws Ics Icg Ict;

% x=[h;sigma;gamma;gamma_dot;Omega];

h=x([1:3]);sigma=x([4:6]);gamma=x([7:9]);gamma_dot=x([10:12]);Omega=x([13:15]);

% reference input ----------------------------------
w_r=zeros(3,1);
sigma_r=zeros(3,1);
w_r_dot=zeros(3,1);
% --------------------------------------------------

% Attitude and rate rrror --------------------------
sigma_e = sigma;
% --------------------------------------------------


As=As0*diag(cos(gamma))+At0*diag(sin(gamma));
At=At0*diag(cos(gamma))-As0*diag(sin(gamma));
Ag=Ag0;

J=I_B + As*Ics*As' + At*Ict*At' + Ag*Icg*Ag';
w=inv(J)*(h-Ag*Icg*gamma_dot-As*Iws*Omega);

g_e=zeros(3,1);
h_d=cross(h,w)+g_e;

% Controller ---------------------------
%  gamma_ddot, Omega_dot
% --------------------------------------

es1=As(:,1);es2=As(:,2);es3=As(:,3);
et1=At(:,1);et2=At(:,2);et3=At(:,3);

D=As*Iws;

C=At*Iws*diag(Omega)+0.5*[(es1*et1'+et1*es1')*(w+w_r),(es2*et2'+et2*es2')*(w+w_r),(es3*et3'+et3*es3')*(w+w_r)]*(Ics-Ict);
Q=[C D];

% Weighting Matrix -------------------------
%W=eye(6);
W=[eye(3) zeros(3,3);
   zeros(3,3) eye(3)*1e-10];
% ------------------------------------------

% Feedback Gains ---------------------------
global k0 k1 k2;
k0=10.0;
k1=25.0*eye(3);
k2=10.0*eye(3);
% ------------------------------------------

% Required Control torque Lr --------------------------
Lr=k1*(w-w_r)+k0*sigma_e-J*w_r_dot+cross(h,w)+g_e;

% power tracking ----------------------------------
P=100;
Pm=P-[zeros(1,3) Omega'*Iws]*W*Q'*inv(Q*W*Q')*Lr;
Proj=eye(6)-W*Q'*inv(Q*W*Q')*Q;

eta_n_dot=Proj*[zeros(3,1);Iws*Omega]*Pm/([zeros(1,3) Omega'*Iws]*Proj*[zeros(3,1);Iws*Omega]);

%if t<30
%   eta_n_dot=0;
%end

eta_d_dot=W*Q'*inv(Q*W*Q')*Lr + eta_n_dot;

%gamma_dot,eta_d_dot,

temp=zeros(6,1);
temp=[zeros(3,3) eye(3);k2*eye(3) zeros(3,3)]*(eta_d_dot-[gamma_dot;zeros(3,1)]);

Omega_d=temp(1:3);
P=Omega'*Iws*Omega_d;
gamma_dot_d=temp(4:6);
gamma_d=gamma_dot;

sigma_cross=[0 -sigma(3) sigma(2); sigma(3) 0 -sigma(1); -sigma(2) sigma(1) 0];
sigma_d=0.5*(eye(3)+sigma_cross+sigma*sigma'-eye(3)*(1+sigma'*sigma)/2)*w;


xd=[h_d;sigma_d;gamma_d;gamma_dot_d;Omega_d];

global flag;

if flag==200
   t,P,flag=0;
end
flag=flag+1;

      
      




