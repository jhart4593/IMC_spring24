%% #2
clear;clc;close all;

% System parameters
R1 = 1;
R2 = 3;
C1 = 0.05;
C2 = 0.025;
R3 = 5;
L = 1;

% State space form
A = [-1/(C1*R2*(1+R1/R2)) 0 -1/L;
    0 0 1/L
    1/C1 -1/C2 -R3/L];
B = [1/(R2*(1+R1/R2));0;0];
C = [1/C1 0 0];
D = 0;
sys = ss(A,B,C,D);

% Check for stability and controllability
isstable(sys)  % same as checking for negative real values for eigs of A
rank(ctrb(sys))

% LQR part a
% first system
Qa1 = [1 0 0;
     0 1 0;
     0 0 1];
Ra1 = 0.01;
Ka1 = lqr(sys,Qa1,Ra1);
sysa1 = ss((A-B*Ka1),B,C,D);

% second system
Qa2 = [1 0 0;
     0 1 0;
     0 0 1];
Ra2 = 10;
Ka2 = lqr(sys,Qa2,Ra2);
sysa2 = ss((A-B*Ka2),B,C,D);

% LQR part b
% first system
Qb1 = [100 0 0;
     0 1 0;
     0 0 10];
Rb1 = 1;
Kb1 = lqr(sys,Qb1,Rb1);
sysb1 = ss((A-B*Kb1),B,C,D);

% second system
Qb2 = [1 0 0;
     0 1 0;
     0 0 10];
Rb2 = 1;
Kb2 = lqr(sys,Qb2,Rb2);
sysb2 = ss((A-B*Kb2),B,C,D);

% Response to initial conditions
x0 = [0.01 0 0]; % initial conditions
initial(sysa1,'r--',sysa2,'b',sysb1,'g-.',sysb2,'k',x0)
title('Response to Initial Conditions');
xlabel('Time (s)');
ylabel('Voltage Across C1 (V)');
legend('a1','a2','b1','b2');

%% #3
clear;clc;close all;

% System parameters
m = 1;
R = 1;
I = 1;
k = 1;

% State space form
A = [0 1/m;
    -k/(1+I/(m*R^2)) 0];
B = [0;1/(1+I/(m*R^2))];
C = [1 0];
D = 0;
sys = ss(A,B,C,D);

% Check for stability and controllability
isstable(sys)  % same as checking for negative real values for eigs of A
rank(ctrb(sys))

% first system LQI
Q1 = [1 0 0;
     0 1 0;
     0 0 1];
R1 = 1;
K1 = lqi(sys,Q1,R1);
A1a = [A [0;0];-C 0];
B1a = [B;0];
C1a = [C 0];
B1i = [0;0;1];
sys1 = ss((A1a-B1a*K1),B1i,C1a,D);

% second system LQI
Q2 = [0.1 0 0;
     0 0.1 0;
     0 0 0.1];
R2 = 1;
K2 = lqi(sys,Q2,R2);
A2a = [A [0;0];-C 0];
B2a = [B;0];
C2a = [C 0];
B2i = [0;0;1];
sys2 = ss((A2a-B2a*K2),B2i,C2a,D);

% Plot system response to unit step input force w/ 0 IC
% time domain
f1 = figure;
step(sys1,'r--',sys2,'b')
legend('sys1','sys2')

% frequency domain
f2 = figure;
bode(sys1,'r--',sys2,'b')
legend('sys1','sys2')

% Response to initial conditions
f3 = figure;
x0 = [1 0 0]; % initial conditions
initial(sys1,'r--',sys2,'b',x0)
title('Response to Initial Conditions');
xlabel('Time (s)');
ylabel('Xc (dist)');
legend('sys1','sys2');
