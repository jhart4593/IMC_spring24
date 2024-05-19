%% 5-1

num = [0 2 1];
den = [1 7 25];
sys = tf(num,den)

[A,B,C,D] = tf2ss(num,den)

%% 5-2

num = [0 0 10 1];
den = [1 9 14 0];
sys = tf(num,den)

[A,B,C,D] = tf2ss(num,den)

%% 4

syms L R1 R2 C s 

A = [-R2/L 1/C;-1/L -1/(R1*C)];
B = [0;1/R1];
C = [R2/L 0];
D = 0;

sIA = s*eye(2)-A;

sys = (C*adjoint(sIA)*B)/det(sIA)