% 1. Controller
K_p=1;
K_i=0; % recorded 0.04
K_d=0; % recorded 0.25
T_f=0; % recorded 10^4

G_controller = pid(K_p,K_i,K_d,T_f);
tf(G_controller)

% 2. Pump
K_pump = 0.0000035;
G_pump = zpk([],[],K_pump);
tf(G_pump)
% 3. Tank Models (this uses the values from lab 3)

g = 9.81;
A_1 = 1.59e-3;
A_01 = 1.96e-5; 
A_2 = 1.59e-3;
A_02 = 1.263-5; 
C_d1 = 0.662;
C_d2 = 0.708;
h_10 = 15e-2;
h_20 = 7e-2;

G_1=tf([1/A_1],[1 (A_01*C_d1*sqrt(2*g))/(A_1*2*sqrt(h_10))]);
tf(G_1)
G_2=tf([1/A_2],[1 (A_02*C_d2*sqrt(2*g))/(A_2*2*sqrt(h_20))]);
tf(G_2)

% 4. Feedback System
T=feedback((G_controller*G_pump*G_1*G_2),1);
tf(T)

% 5. Calculate tank 1 height if tank 2 is at 20cm
g = 9.81;
A_1 = 1.59e-3;
A_01 = 1.96e-5; 
A_2 = 1.59e-3;
A_02 = 1.263e-5; 
C_d1 = 0.662;
C_d2 = 0.708;

h_20 = 0.10;

% Steady-state flows and h1
Qo2 = A_02*C_d2*sqrt(2*g*h_20);
Qo1 = Qo2;
h10 = (Qo1/(A_01*C_d1))^2/(2*g);

G_1=tf([1/A_1],[1 (A_01*C_d1*sqrt(2*g))/(A_1*2*sqrt(h_10))]);
tf(G_1)
G_2=tf([1/A_2],[1 (A_02*C_d2*sqrt(2*g))/(A_2*2*sqrt(h_20))]);
tf(G_2)
T=feedback((G_controller*G_pump*G_1*G_2),1);
tf(T) % Feedback system with this operating point
fprintf('h_{1,0} = %.5f m (%.2f cm)\n', h_10, 100*h_10);

% 6. Step Resopnse
S=1-T;
G_vp_over_r=minreal(G_controller*S); % Controller Output

% sim over 0.001 m step
tEnd = 300;
dt = 0.1;
t = 0:dt:tEnd;

% --- delayed step reference: 0.01 m starting at 50 s
u = zeros(size(t));
u(t >= 50) = 0.01;         % step starts at 50s

% --- system responses
[y,~] = lsim(T, u, t);                % δh2 response
uVp    = lsim(G_controller*(1-T), u, t);         % δVp = C*S*u, with S = 1-T


% plot step resonse
figure;
subplot(2,1,1);
plot(t,y,'LineWidth',1.2);
grid on;
ylim([-0.005 0.025]); 
ylabel('\delta h_2 (m)');
title('Closed loop resonse to 1 cm step in tank 2 height')

subplot(2,1,2)
plot(t,u,"LineWidth",1.2);
ylim([-0.01 0.02]); 
grid on;
ylabel('\delta V_p (V)');
xlabel('Time (s)')

