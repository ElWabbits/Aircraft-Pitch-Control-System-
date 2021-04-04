%% EE 471 Final Project: Aircraft Pitch
% 
%  Name: Brandon Serna, Grace Hutchison, Erik Perales
%  Student ID: 011956215
%  Date:  12/20/2020

%% Aircraft Pitch: System Modeling
clear clc 
close all
clearvars

% Transfer Function of the System
s = tf('s');
P_aircraft = (1.151*s + 0.1774)/(s^3 + .739*s^2 +0.921*s);
% Transfer Function of the aircraft pitch system

inputs = {'S(delta)'};
outputs = {'theta'};

set(P_aircraft,'InputName',inputs)
set(P_aircraft,'OutputName',outputs)

P_aircraft

%%
% State-Space Representation of the System

A = [ -0.313   56.7 0;
      -0.0139 -0.426 0;
           0   56.7 0];
% System Matrix

B = [0.232; 0.0203; 0];
% Input Matrix

C = [0 0 1];
% Output Matrix

D = 0;
% Feedforward Matrix

states = {'alpha' 'q' 'theta'};
inputs = {'S(delta)'};
outputs = {'theta'};
sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs)
aircraft_pitch = tf(sys_ss)

%% Aircraft Pitch: System Analysis
clear clc 
close all
clearvars
s = tf('s');
P_aircraft = (1.151*s + 0.1774)/(s^3 + .739*s^2 +0.921*s)
% Transfer Function of the aircraft pitch system
%%
% Open-Loop Response

t = [0:0.01:10];
figure()
step(0.2*P_aircraft,t);
axis([0 10 0 0.8]);
ylabel('Pitch Angle(rad)');
title('Open-loop Step Response of Aircraft Pitch');
% Plots the open-loop step response of the system
Poles = pole(P_aircraft)
% Poles of the open-loop aircraft pitch system
Zeros = zero(P_aircraft)
% Zeros of the open-loop aircraft pitch system

%%
% Closed-Loop Response
cl_sys = feedback(P_aircraft,1)
% Closed-Loop TF of the aircraft pitch system
figure()
step(0.2*cl_sys);
ylabel('Pitch Angle(rad)');
title('Closed-loop Step Response of Aircraft Pitch');
Poles = pole(cl_sys)
Zeros = zero(cl_sys)
% Poles and zeros of the closed-loop aircraft pitch system
stepinfo(0.2*cl_sys)
%%
% Closed-Loop Response in Time Domain
s = tf('s');
R = 0.2/s;
% Step w/ a magnitude of 0.2
Ys = zpk(cl_sys*R);
% System Output due to a step
[r,p,k] = residue(0.2*[1.151 0.1774],[1 0.739 2.072 0.1774 0])
[num,den] = residue(r(1:2),p(1:2),k);
tf(num,den);
syms s
Ys = (0.2/s) - (0.0881/(s+0.08805)) - ((0.1121*s + 0.08071)/(s^2 + 0.6509*s +2.015));
% Partial Fraction Expansion of our system
yt = ilaplace(Ys)
% Calculating the time domain represenation of our system 
t = [0:0.1:70];
yt = 0.2 - 0.0881*exp(-0.08805*t) - exp(-0.3255*t).*(0.1121*cos(1.3816*t)+0.0320*sin(1.3816*t));
figure()
plot(t,yt)
xlabel('Time(sec)');
ylabel('Pitch Angle(rad)');
title('Closed-loop Step Response of Aircraft Pitch');

%% Aircraft Pitch: PID Controller Design
clear clc 
close all
clearvars

s = tf('s');
P_aircraft = (1.151*s + 0.1774)/(s^3 + .739*s^2 +0.921*s);
% Plant of our aircraft pitch system
controlSystemDesigner(P_aircraft);

%% Aircraft Pitch: Root Locus Design
clear clc 
close all
clearvars

s = tf('s');
P_aircraft = (1.151*s + 0.1774)/(s^3 + .739*s^2 +0.921*s);
%controlSystemDesigner('rlocus',P_aircraft);
Cp = 900*(s+1.8)/(s+65);
Ls = P_aircraft*Cp;
Ts = feedback(Ls,1);
stepinfo(0.2*Ts)
%% Aircraft Pitch: Frequency Domain Design
clear clc 
close all
clearvars
s = tf('s');
P_aircraft = (1.151*s + 0.1774)/(s^3 + .739*s^2 +0.921*s);
% Plant of our aircraft pitch system
figure()
margin(P_aircraft)
% Frequency Response of the Open-Loop system
grid
Ts = feedback(P_aircraft,1);
% Closed-Loop Transfer Function of our system
figure()
step(0.2*Ts);
% Closed-Loop Step Response of our system 
%%
% Lead Compensator Design
K = 10;
figure()
margin(K*P_aircraft) 
grid
Ts = feedback(K*P_aircraft,1);
figure()
step(0.2*Ts), 
grid
title('Closed-Loop Step Response of Aircraft Pitch with K = 10')

%%
%
OS = 5;
K = 10;
zeta = (-log(OS/100))/(sqrt(pi^2 + (log(OS/100))^2))
phase_marg = zeta*100
% Desired PM
alpha = (1-sind(65))/(1+sind(65))
mag_wc = 20*log10(1/sqrt(alpha))
% @ -13dB, wm = 7.41
wm = 7.41
% New crossover frequency
T = 1/(wm*sqrt(alpha))
Cs= K*(T*s + 1)/(alpha*T*s + 1)
% Lead Compensator
figure()
margin(Cs*P_aircraft)
grid
sys_cl = feedback(Cs*P_aircraft,1);
figure()
step(0.2*sys_cl)
xlim([0 10])
grid
title('Closed-loop Step Response of Aircraft Pitch with Lead Compensator')
stepinfo(0.2*sys_cl)

%%
%
alpha = (1-sind(70))/(1+sind(70))
mag_wc = 20*log10(1/sqrt(alpha))
% @-15db, wmax is approximatley 8.2
wm = 8.2
T = 1/(wm*sqrt(alpha))
Cs= K*(T*s + 1)/(alpha*T*s + 1)
% New lead compensator
figure()
margin(Cs*P_aircraft)
grid
sys_cl = feedback(Cs*P_aircraft,1);
figure()
step(0.2*sys_cl)
xlim([0 5])
grid
title('Closed-loop Step Response of Aircraft Pitch with Lead Compensator')
stepinfo(0.2*sys_cl)

%% Aircraft Pitch: State Space Methods
clear clc 
close all
clearvars

% State-Space Representation of the System
A = [ -0.313   56.7 0;
      -0.0139 -0.426 0;
           0   56.7 0];
% System Matrix
B = [0.232; 0.0203; 0];
% Input Matrix
C = [0 0 1];
% Output Matrix
D = 0;
% Feedforward Matrix
air_pitch = ss(A,B,C,D)
% State-Space Model of our Aircraft Pitch system
Co = ctrb(air_pitch)
% Controllability Matrix of our system
rank(Co)
% Rank of our controllability matrix


%%
% Pole Placement Implementation

Ts = 1;   % Settling Time
OS = 5;  % Percent Overshoot
zeta = (-log(OS/100))/(sqrt(pi^2 + (log(OS/100))^2));
wn = 4/(zeta*Ts);

P = [-20+20.4i -20-20.4i -0.154];
% Desired closed-loop pole placements
K = place(A,B,P)
% Gain matrix
Ac = [(A-B*K)];
N = 1/(C*((-Ac)^-1)*B)
% Precompensation Gain
cl_sys = ss(Ac,N*B,C,D);
% State-Space Model of Closed-Loop System 
step(.2*cl_sys);
ylabel('Pitch Angle(rad)');
title('Closed-Loop Step Response of Aircraft Pitch System: Pole Placement');
stepinfo(cl_sys)
%%
% LQR Implementation
p = 55;
% State-Cost Weighting Factor
Q = p*C'*C;
R = 1;
% Control Weighting Factor
K = lqr(A,B,Q,R)
% Gain matrix
Ac = [(A-B*K)];
kr = 1/(C*((-Ac)^-1)*B)
% Prefilter
cl_sys = ss(Ac,kr*B,C,D);
step(0.2*cl_sys)
ylabel('Pitch Angle(rad)');
title('Closed-Loop Step Response of Aircraft Pitch System: LQR');
stepinfo(0.2*cl_sys)



