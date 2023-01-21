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
    0 0 -(m+M*L)*g/(M*L) 0];
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

poles = eig(A);      % pole
transFc = tf(sys);   % transfer_func of system 

%% check observability
Rb = obsv(A,C) ;
ROb = rank(Rb)

%% system
sys=ss(A,B,C,D);
x0=[0;5;0;0];

%% LQR Controller
Q = [50 0 0 0;0 1 0 0;0 0 50 0;0 0 0 1];
R = 1;
K_d = lqr(A,B,Q,R)

%% Observer
E = 5*eig(A-B*K_d);
L_o = (place(A',C',E))'