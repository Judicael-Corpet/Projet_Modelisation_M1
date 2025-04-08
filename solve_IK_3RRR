clear all
close all

global L1 L2 Rb Re

L1=0.10;  %long segment 1
L2=0.10;  %long segment 2
Rb=0.1322594;  % Rayon base
Re=0.07; % Rayon effecteur

pos_eff=[0.0, 0.0, 0]; % pose effecteur

% 1ière méthode : résolution de systèmes d'éq non-linéaires 
% solutions initiales des angles alpha beta des bras 1,2,3
q0=[0; pi/2; 0; pi/2; 0; pi/2];

q=fsolve(@(q) solve_eq_NL(q,pos_eff),q0)

trace_rob(q);

% 2ième méthode : résolution analytique du MGI 2R plan :renvoie alphi_i et
% beta_i
q=MGI_analytique(pos_eff)

trace_rob(q);
