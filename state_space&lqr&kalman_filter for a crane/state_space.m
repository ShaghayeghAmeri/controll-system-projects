clc;clear ;close all
%% define parameters
M = 0.5;
m = 0.1;
L = 0.6;
b = 0.25;
g = 9.81;

%% define the matrices
A=[0 1 0 0;
    0 -b/M m/M*g 0;
    0 0 0 1;
    0 0 -(m+M*L)*g/(M*L) 0]
B=[0;
   1/M;
   0;
   -1/(M*L)];
C = [1 0 0 0;
     0 0 1 0];
D = [0];

 
states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'x','phi'};

sys = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);


poles = eig(A)     % pole
transFc = tf(sys) % transfer_func of system 
K=eig(A)

%% Diagrams
%num=[2  4.441e-16 19.62];
%den=[1 0.5 13.08 6.54];
%rlocus(tf(num,den))
num=[-3.333];
den=[2 13.08];
rlocus(tf(num,den))
%% check controllability and observability
C0 = ctrb(A,B) ;
Rc = rank(C0);
Rb = obsv(A,C) ;
ROb = rank(Rb); 
