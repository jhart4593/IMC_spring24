%% 1A
clear;clc

p = [1 8 17 10]
roots(p)

%% 1B
clear;clc

p = [1 2 3 2]
roots(p)

%% 2
clear;clc;

A = [0 1 0;0 0 1;-5 -3 -6]
B = [0 0 1]'

two = A*B
three = A^2*B

Pc = [B two three]

rank(Pc)
det(Pc)

ctrb(A,B)
%% 3
clear;clc;

syms s
A = [1 3;-4 -7]
t = s*eye(2)-A
in = inv(t)
ilaplace(in)
%% 4
clear;clc;

syms s
A = [0 2;-5/2 -2]
t = s*eye(2)-A
in = inv(t)
roots([1 2 5])
ilaplace(in)

%% 5a
clear;clc

syms h
A = [1 -3 3;3 -5 3;6 -6 4]
t = h*eye(3)-A
ce = det(t)
roots([1 0 -12 -16])

%% 5b
clear;clc;
format rat
syms s t real

A = [1 -3 3;3 -5 3;6 -6 4]
x = [1 -2 4;0 1 -4;1 4 16]
alpha = inv(x)*[exp(-2*t) t*exp(-2*t) exp(4*t)]'
a0 = alpha(1,:)
a1 = alpha(2,:)
a2 = alpha(3,:)

stm = a0*eye(3)+a1*A+a2*A^2

h = s*eye(3)-A;
test = ilaplace(inv(h));
isequal(stm,test)
%% 6a
clear;clc

syms h
A = [-1 2 0;0 1 8;0 -1 -3]
t = h*eye(3)-A
ce = det(t)
roots([1 3 7 5])

%% 6b
clear;clc;
format rat
syms s t real

A = [-1 2 0;0 1 8;0 -1 -3]
x = [1 -1 1;1 -1 -3;1 2 -4]
alpha = inv(x)*[exp(-t) exp(-t)*cos(2*t) exp(-t)*sin(2*t)]'
a0 = alpha(1,:)
a1 = alpha(2,:)
a2 = alpha(3,:)

stm = a0*eye(3)+a1*A+a2*A^2
new_stm = simplify(stm)

h = s*eye(3)-A;
test = ilaplace(inv(h))
isequal(new_stm,test)
