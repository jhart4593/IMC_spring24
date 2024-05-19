%% 3b
clear,clc;

num = [0 0 2];
den = [1 2 0];
sys = tf(num,den);

rlocus(sys)

%% 4
clear,clc;

Kp = 8;
Kd = 3;
a = 2;

Cs = tf([0 Kd Kp],1);
Gs = tf([0 0 a],[1 a 0]);
OLTF = Cs*Gs;
CLTF = feedback(OLTF,1);

step(CLTF)
S = stepinfo(CLTF)

