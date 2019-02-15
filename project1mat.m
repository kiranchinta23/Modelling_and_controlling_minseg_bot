%% problem1
% 1.1 matlab script to convert the minseg dynamic model to statespace
clc;
Mp=0.285;  % pendulam mass
Mw=0.025;  % Wheel mass
Jp=0.0010; % Pendulam inertia
Jw=0.0013; % Wheel Inertia
rw=0.022;  % Wheel radius
L=0.1;     % pendulam length
c=0.0001;  % Rotational damping
Kt=0.3;    % Torque constant
Kb=0.5;    % Back-emf constant
R=5;       % coil resistance
g=9.81;    % gravitational resistance

M=[(Jw/rw)+(Mw*rw)+(Mp*rw) (rw*Mp*L);(Mp*L) (Jp+Mp*L^2)];
D=[((c/rw)+(Kt*Kb)/(rw*R)) 0;-((c/rw)+(Kt*Kb)/(rw*R)) 0];
K=[0 0; 0 -Mp*g*L];
F=[Kt/R; -Kt/R];


A=[zeros(2) eye(2); -inv(M)*K -inv(M)*D];
B=[0; 0; inv(M)*F];
C=eye(4);
DS=[0 0 0 0]';

%1.2 
eigne=eig(A);
display(eigne);

%1.3
IN=[0;(3.14/60);0;0];
sim('E:\career\Mechanical\fall-17\ME 190 MECHATRONICS\project\ME190project1.slx');
open_system('E:\career\Mechanical\fall-17\ME 190 MECHATRONICS\project\ME190project1.slx');

plot(time,output);
legend('positon','anglar positon','velocity','angular velocity');
xlabel('time')
ylabel('output')

% according to physics it make sense, that while falling minseg robot
% angular velocity, angular position increases but velocity and position
% reaches negative values to make robot balance itself.

%% Part 2:
%2.1 matab script
P=[-9;-9;-9;-9];
K_acker=acker(A,B,P)
IN=[0;(31.4/180);0;0];
sim('E:\career\Mechanical\fall-17\ME 190 MECHATRONICS\project\ME190project1P2.slx');
open_system('E:\career\Mechanical\fall-17\ME 190 MECHATRONICS\project\ME190project1P2.slx');

eige=eig(A)
figure();plot(real(eige),imag(eige),'o')
xlabel('Real axis');
ylabel('Imaginary axis');
title('Location of eigen values of open loop system');

Y=eig(A-B.*K_acker)
figure();plot(real(Y),imag(Y),'*');
xlabel('Real axis');
ylabel('Imaginary axis');
title('Location of eigen values of closed loop system');

for n=3:2:9
    IN=[0;(n*3.14/180);0;0];
sim('E:\career\Mechanical\fall-17\ME 190 MECHATRONICS\project\ME190project1P2.slx')
figure();plot(time,output)
legend('positon','angular position','velocity','angular velocity');
xlabel('time');
ylabel('response');
title('Response of the system for different inital conditions');

figure();plot(time,volts);
legend('volts');
xlabel('time');
ylabel('volts');
title('control inpout of the system for different initial conditions');
end

%% Part 3 
% 3.1 matlab script
v=[1 100 1 100];
Q=diag(v);
R=[1];
K_lqr=lqr(A,B,Q,R);
IN=[0;(31.4/180);0;0];
sim('E:\career\Mechanical\fall-17\ME 190 MECHATRONICS\project\ME190project1P3.slx');
open_system('E:\career\Mechanical\fall-17\ME 190 MECHATRONICS\project\ME190project1P3.slx');

figure();plot(time,output)
legend('positon','angular position','velocity','angular velocity');
xlabel('time');
ylabel('response');
title('Response of the system for ten degrees inital conditions');

figure();plot(time,volts);
legend('volts');
xlabel('time');
ylabel('volts');
title('control inpout of the system for ten degrees initial conditions');

Y=eig(A-B.*K_lqr)
figure();plot(real(Y),imag(Y),'*');
xlabel('Real axis');
ylabel('Imaginary axis');
title('Location of eigen values of closed loop system');


for n=3:2:7
    IN=[0;(n*3.14/180);0;0];
sim('E:\career\Mechanical\fall-17\ME 190 MECHATRONICS\project\ME190project1P3.slx')
figure();plot(time,output)
legend('positon','angular position','velocity','angular velocity');
xlabel('time');
ylabel('response');
title('Response of the system for different inital conditions');

figure();plot(time,volts);
legend('volts');
xlabel('time');
ylabel('volts');
title('control inpout of the system for different initial conditions');
end





