%------------------------------------------
% Initialization for sim01.m
%------------------------------------------

% -- define the CMG axis's --------------
e_g10=[0;1;0];e_g20=[0;0;1];e_g30=[1;0;0];
e_s10=[1;0;0];e_s20=[0;1;0];e_s30=[0;0;1];
e_t10=cross(e_g10,e_s10);e_t20=cross(e_g20,e_s20);e_t30=cross(e_g30,e_s30);

global As0 At0 Ag0;
As0=[e_s10,e_s20,e_s30];
At0=[e_t10,e_t20,e_t30];
Ag0=[e_g10,e_g20,e_g30];


% -- define inetia matrix ---------------
global I_B Iws Ics Icg Ict;
I_B=diag([86.2,85.1,113.6]);
Iws=diag([0.05,0.05,0.05]);
Iwt=diag([0.03,0.03,0.03]);Iwg=Iwt;
Igt=diag([0.01,0.01,0.01]);
Igs=diag([0,0,0]);Igg=Igs;
Ics=Iws+Igs;Icg=Iwg+Igg;Ict=Iwt+Igt;

% -- initial gimbal angle/rate
gamma=[0;0;0]; gamma_dot=[0;0;0];

%  -- initial wheel rate
Omega=[144;144;144]; 

% -- initial body attitude/rate
sigma=[-0.3;-0.2;0.6]*.5;
w=[0;0;0]; % rest-start ---------------
%h=[e_s10 e_s20 e_s30]*Iws*Omega; 
h=As0*Iws*Omega;

