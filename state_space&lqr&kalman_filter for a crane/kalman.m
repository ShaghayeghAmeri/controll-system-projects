clc;clear ;close all
%% define parameters
M = 0.5;
m = 0.1;
kF = 0.6;
b = 0.25;
g = 9.81;

%% define the matrices
A=[0 1 0 0;
    0 -b/M m/M*g 0;
    0 0 0 1;
    0 0 -(m+M*kF)*g/(M*kF) 0];
B=[0;
   1/M;
   0;
   -1/(M*kF)];
C = [1 0 0 0;];
D = zeros(size(C,1),size(B,2));

%%  Augment system with disturbances and noise
Vd =0.1*eye(4);  % disturbance covariance
Vn = 1;       % noise covariance

BF = [B Vd 0*B];  % augment inputs to include disturbance and noise

sys_NOISE = ss(A,BF,C,[0 0 0 0 0 Vn]);  % build big state space system... with single output

sysFullOutput = ss(A,BF,eye(4),zeros(4,size(BF,2)));  % system with full state output, disturbance, no noise

%%  Build Kalman filter
[kF,P,E] = lqe(A,Vd,C,Vd,Vn);  % design Kalman filter
KF = (lqr(A',C',Vd,Vn))';   % alternatively, possible to design using "LQR" code
sysKF = ss(A-kF*C,[B kF],eye(4),0*[B kF]);  % Kalman filter estimator

%%  Estimate linearized system in "down" position (Gantry crane)
dt = .01;
t = dt:dt:50;
uDIST = randn(4,size(t,2));
uNOISE = randn(size(t));
u = 0*t;
u(100:120) = 100;     % impulse
u(1500:1520) = -100;  % impulse
uAUG = [u; Vd*Vd*uDIST; uNOISE];

[y,t] = lsim(sys_NOISE,uAUG,t);
[xtrue,t] = lsim(sysFullOutput,uAUG,t);


[x,t] = lsim(sysKF,[u; y'],t);

%plot(t,xtrue,'-',t,x,'--','LineWidth',3)

figure
plot(t,y)
hold on
plot(t,xtrue(:,1),'r','LineWidth',1.5)
plot(t,x(:,1),'k--','LineWidth',2)