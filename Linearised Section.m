% 1. Controller
K_p=1;
K_i=0.04;
K_d=0.25;
T_f=10^4;

G_controller = pid(K_p,K_i,K_d,T_f);
tf(G_controller)

% 2. Pump
K_pump = 0.0000035;
G_pump = zpk([],[],K_pump);
tf(G_pump)
% 3. Tank Models

g = 9.81;
A_1 = 1.59e-3;
A_01 = 1.96e-5; 
A_2 = 1.59e-3;
A_02 = 1.263-5; 
C_d1 = 0.662;
C_d2 = 0.708;
h_10 = ;
h_20 = ;

% 4. Feedback


% 5.
%}


% 6. Step Resopnse