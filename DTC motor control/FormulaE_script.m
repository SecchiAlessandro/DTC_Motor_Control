clc
clear all

%% DC machine parameters
load('Traction_e402B_online');
%parameters of 1 motor = equivalent motor
V_batt = 800; %voltage of battery
pp=4; %pole pairs of each motor
cos_fi=0.85; %power factor of each motor
Rs=0.01; %p.u.
Rr=0.01; %p.u.
X_lock=0.15; %p.u.
X_noload=2.5; %p.u.
d=457*10^-3; %diameter wheels
mass_tot=800; %kg
fr=0.6; %friction coefficient
tr=1;

v_max=200*1000/3600; %max speed m/s
omega_max=v_max/d*2; %rad/s
eff=0.9; %efficiency 

J=mass_tot*v_max^2/omega_max^2;
%J_eq=mass_tot*d^2/4;

Pn=200*10^3; %mechanical power [W]
Pe_tot=Pn/eff;

omega_b=65*1000/3600/d*2; %rad/s base speed
Tn=Pn/omega_b;
psi_r_max=V_batt/(2*pi*50);
i_max=Tn/pp/eff/psi_r_max;
Z=V_batt/i_max;

Rs_eq=Rs*Z; 
Rr_eq=Rr*Z; 
X_lock_eq=X_lock*Z; 
X_noload_eq=X_noload*Z;
Rks=Rs_eq+Rr_eq;
Lks=X_lock_eq/(2*pi*50);
M=X_noload_eq/(2*pi*50);



B=fr;
tau_O=J/B;

SWT_cmplx=cat(3,[exp(j*4*pi/3) exp(j*5*pi/3) 1 exp(j*pi/3) exp(j*2*pi/3)...
    exp(j*pi);0 0 0 0 0 0; exp(j*2*pi/3) exp(j*pi) exp(j*4*pi/3)...
    exp(j*5*pi/3) 1 exp(j*pi/3)],...
    [exp(j*5*pi/3) 1 exp(j*pi/3) exp(j*2*pi/3) exp(j*pi) exp(j*4*pi/3);...
    0 0 0 0 0 0; exp(j*pi/3) exp(j*2*pi/3) exp(j*pi) exp(j*4*pi/3)...
    exp(j*5*pi/3) 1]);
b_tau=[-1, 0, 1];
b_psi=[0,1];
b_s=[-pi/6 pi/6 pi/2 5*pi/6 7*pi/6 3*pi/2];

SWT_conf=cat(3,[5 6 1 2 3 4; 7 8 7 8 7 8; 3 4 5 6 1 2],...
    [6 1 2 3 4 5; 8 7 8 7 8 7; 2 3 4 5 6 1]);


a_max=100/3.6/2.8; %m/s
T_max=mass_tot*a_max*d/2*tr;



T_ref = d/2*tr*traction(:,2).*40;
k=T_ref(1)/psi_r_max;
psi_rd_ref = T_ref./k;

speed_ref = traction(:,1);
%%  PI controller design parameters
s=tf('s');

%GO
tau_O_desired=tau_O/10000;
wc_O=2*pi/tau_O_desired;

%tf
GO = 1/(B+J*s);

%% Zero Pole cancellation (90 phase margin)
% %PI parameters ia
% kp_a=wc_a*La;
% ki_a=wc_a*Ra;
% Regi=kp_a+ki_a/s
% Ti_a=kp_a/ki_a;
% %tf open loop
% Li=Regi*Gi;
% %tf close loop
% Fi=Li/(1+Li);
% % figure
% % bode(Li)
% % figure
% % bode(Fi)
% %PI parameters ie
% kp_e=wc_e*Le;
% ki_e=wc_e*Re;
% Rege=kp_e+ki_e/s
% Ti_e=kp_e/ki_e;
% %tf open loop
% L_e=Rege*Ge;
% %tf close loop
% Fe=L_e/(1+L_e);
% % figure
% % bode(L_e)
% % figure
% % bode(Fe)
% %PI parameters speed
% kp_O=wc_O*J_eq;
% ki_O=wc_O*B;
% RegO=kp_O+ki_O/s
% Ti_O=kp_O/ki_O;
% %tf open loop
% LO=RegO*GO;
% %tf close loop
% FO=LO/(1+LO);
% % figure
% % bode(LO)
% % figure
% % bode(FO)

%% Pidtool
%otherwise use pidtool
phase_m=90;
opt=pidtuneOptions('PhaseMargin', phase_m);

%pidtool(GO)
par_reg_speed=pidtune(GO,'PI',wc_O,opt);
ki_O=par_reg_speed.Ki;
kp_O=par_reg_speed.Kp;

RegO=kp_O+ki_O/s
Ti_O=kp_O/ki_O;
%tf open loop
LO=RegO*GO;
%tf close loop
FO=LO/(1+LO);
% figure
% bode(LO);
% figure
% bode(FO);
% figure
% margin(LO);



