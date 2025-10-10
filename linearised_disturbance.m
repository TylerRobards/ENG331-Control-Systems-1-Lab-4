%% 1) Tuned PIDF (note: pid(Kp,Ki,Kd,N) where N = 1/Tf)
K_p = 1; K_i = 0.04; K_d = 0.25; N = 1e4;     % Tf=1/N=1e-4 s
G_controller = pid(K_p, K_i, K_d, N);

%% 2) Pump
K_pump = 3.5e-6;
G_pump  = zpk([],[],K_pump);

%% 3) Tank models (check A_02 typing!)
g = 9.81;
A_1 = 1.59e-3;  A_01 = 1.96e-5;  C_d1 = 0.662;  h_10 = 15e-2;
A_2 = 1.59e-3;  A_02 = 1.263e-5; C_d2 = 0.708;  h_20 = 7e-2;

G_1 = tf([1/A_1],[1 (A_01*C_d1*sqrt(2*g))/(A_1*2*sqrt(h_10))]);
G_2 = tf([1/A_2],[1 (A_02*C_d2*sqrt(2*g))/(A_2*2*sqrt(h_20))]);

%% 4) Closed-loop maps
L  = G_controller*G_pump*G_1*G_2;
T  = feedback(L,1);             % reference -> delta h2
S  = feedback(1,L);             % sensitivity
Td = minreal(-G_2*S);           % disturbance (flow into Tank 2) -> delta h2

%% 5) Theoretical steady-state error for a step disturbance
D0  = -1e-6;                    % e.g., -1 mL/s step (negative inflow)
yss = dcgain(Td)*D0;            % delta h2 at steady state
ess = -yss;                     % with r=0, e = -y
fprintf('Predicted: y_ss = %.3e m,  e_ss = %.3e m (should be ~0)\n', yss, ess);

%% 6) Time simulation of disturbance-only case (delayed step at t=50 s)
tEnd = 300; dt = 0.1; t = 0:dt:tEnd;
d = zeros(size(t)); d(t>=50) = D0;     % additive step flow at Tank 2 input
y_d = lsim(Td, d, t);                  % output due to disturbance
e_d = -y_d;                            % error when r(t)=0

figure;
subplot(3,1,1); plot(t,d,'LineWidth',1.2); grid on;
ylabel('d (m^3/s)'); title('Step disturbance at Tank 2 input');

subplot(3,1,2); plot(t,y_d,'LineWidth',1.2); grid on;
ylabel('\delta h_2 (m)'); title('\delta h_2 due to disturbance');

subplot(3,1,3); plot(t,e_d,'LineWidth',1.2); grid on;
ylabel('e(t) (m)'); xlabel('Time (s)'); title('Tracking error (r=0)');