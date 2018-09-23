close all
clear
clc
%%% Annoying laws of motion part
%%% Initiate variables
g = 9.81;
bf = 0;
mb = 0.381;
lb = 0.112;
Ib = 0.00616;
mw = 0.036;
lw = 0.021;
Iw = 0.00000746;
Rm = 4.4;
Lm = 0;
bm = 0;
Ke = 0.444;
Kt = 0.470;

%%% Substitutions
Km = Ke*Kt/Rm+bf;
I_eq = Ib+lb^2*mb;
m_eq = Iw/(lw^2)+mb+mw;

Gamma = [lw*m_eq, lw*lb*mb;
         lb*mb,   I_eq];
Alpha = [0, -Km/lw, 0, Km;
         0, Km/lw, g*lb*mb, -Km];
Beta  = [Kt/Rm, 0;
         -Kt/Rm, lb];
A = Gamma\Alpha;
A = [0, 1, 0, 0;
     A(1,:);
     0, 0, 0, 1;
     A(2,:)];
B = Gamma\Beta;
% B = [0, 0;
%      B(1,1), -lb^2*mb/(I_eq*m_eq);
%      0, 0;
%      B(2,1), lb/I_eq];
B = [0;
     B(1,1);
     0;
     B(2,1)];
C = [0, 0, 1, 0];
D = 0;

G_analytical = ss(A,B,C,D);
G = tf(G_analytical);
s = tf('s');

%%% Pole placement
p1 = 80;
p2 = 60;
p3 = 50;

% Calculating gains for pole placement
Ki = (-15390 - p3*p2*p1 ) / 90.03;
Kp = (-62.08 - (p3*p2 + p3*p1 + p2*p1) ) / 90.03;
Kd = ( 475 - (p3 + p2 + p1) ) / 90.03;

% Defining controller C
C = pid(Kp,Ki,Kd,0);

% Define closed-loop system
G_test = -90.03*s / (s^3 + 475*s^2 - 62.08*s - 1.539e4);
T = feedback(C*G,1);

B_for_poking = [0, 0;
                B(1,1), -lb^2*mb/(I_eq*m_eq);
                0, 0;
                B(2,1), lb/I_eq];
C_for_full_observability = eye(4);
D_for_full_observability_and_poking = zeros(4,2);