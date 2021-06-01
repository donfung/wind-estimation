%% Altitude hold 
clc
clear all
close all
[A,B] = longitudinal_Navion_A_B();
u0 = 176; % ft/s
%% From AA 271A HW4
%% Part c - transfer functions
% de,dT to ubar
C = [1 0 0 0];
D = [0 0];
sys = ss(A,B,C,D);
to_ubar = (tf(sys));
tf_de_to_ubar = zpk(to_ubar(1,1));
tf_dT_to_ubar = zpk(to_ubar(1,2));
% de, dT to theta
C = [0 0 0 1];
sys = ss(A,B,C,D);
to_theta = (tf(sys));
tf_de_to_theta = zpk(to_theta(1,1));
tf_dT_to_theta = zpk(to_theta(1,2));
% de, dT to hdot
C = [0 -u0 0 u0]; % y = hdot = u0*theta - w = u0*theta - u0*alpha
sys = ss(A,B,C,D);
to_hdot = (tf(sys));
tf_de_to_hdot = zpk(to_hdot(1,1));
tf_dT_to_hdot = zpk(to_hdot(1,2));

%% Part d - design control law
tf_de_to_h =1/tf('s')*to_hdot(1,1)
fprintf('This matches the given transfer function!\n');
sisotool(tf_de_to_h)
s = tf('s');
comp = -0.035*(s+0.5)/(s+10);
CL_sys = feedback(tf_de_to_h*comp, 1);
% Can view poles with this
damp(CL_sys)

step(CL_sys)
