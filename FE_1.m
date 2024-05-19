%Final Exam Problem 1
clear;clc;close all;

%Parameters
J1 = 1;
J2 = 0.1;
k = 0.091;
b = 0.0036;

%State Space Model
A = [0 1/J1 -1/J2;-k -b/J1 b/J2;k b/J1 -b/J2];
B = [0;1;0];
C = [1 0 0];
D = 0;
sys = ss(A,B,C,D);

%Stability and Controllability
isstable(sys)  % same as checking for negative real values for eigs of A
rank(ctrb(sys))

%Pole placement design------------------------------------------------
%Desired closed loop poles, solve for K
%Find K based on desired characteristic equation
coeff = [1 7.2 20.52 21.6];
P1 = roots(coeff);
K1 = place(A,B,P1);
%Find K using arbitrary pole locations as estimate
P2 = [-1+1i -1-1i -3];
K2 = place(A,B,P2);

%Check stability of closed loop system, also unit step response
Acl1 = A-B*K1;
syscl1 = ss(Acl1,B,C,D);
isstable(syscl1)
[y1,t] = step(syscl1);
plot(t,y1,'r--','LineWidth',2)
hold on

Acl2 = A-B*K2;
syscl2 = ss(Acl2,B,C,D);
isstable(syscl2)
[y2,t] = step(syscl2);
plot(t,y2,'g:','LineWidth',2)
hold on

%LQR design------------------------------------------------------------
Q = [10 0 0;
     0 1 0;
     0 0 1];
R = 0.01;
K_opt = lqr(sys,Q,R);
sys_opt = ss((A-B*K_opt),B,C,D);
isstable(sys_opt)
[y3,t] = step(sys_opt);
plot(t,y3,'b','LineWidth',2)

xlabel('Time (seconds)')
ylabel('Amplitude')
title('Step Response')
grid on
legend('Pole Placement Desired CE','Pole Placement Arbitrary','LQR')