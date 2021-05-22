%% Navion lateral dynamics

% States
% x = [beta, p, r, phi, psi]^T   u = [da, dr]^T 
% x = normalized v vel (beta), roll rate p, yaw rate r, roll angle phi, yaw angle psi
% u = aileron deflection, rudder deflection
% xdot = Ax + Bu

function [A,B] = lateral_Navion_A_B()
    % Aircraft geometrical parameters
    m = 85.4; % slugs
    weight = 2750; % lbs
    g = 32.1741; % ft/s2
    u0 = 176; % ft/s
    S = 184; % ft^2
    qinf = 36.8305; % lb/ft^2
    Ix = 1048; % slug-ft^2
    Iy = 3000; % slug-ft^2
    Iz = 3530; % slug-ft^2
    Jxz = 0; % slug-ft^2
    c = 5.7; % ft
    b = 33.4; % ft

    % Dimensional derivatives
    X_w = 0.03607;
    X_u = -0.0451;
    X_de = 0;
    Z_w = -2.0244;
    Z_u = -0.3697;
    Z_de = -28.17;
    M_w = -0.04997;
    M_wdot = -0.005165;
    M_q = -2.0767;
    M_u = 0;
    M_de = -11.1892;
    Y_v = -0.2543;
    Y_da = 0;
    Y_dr = 0.0708;
    L_beta = -15.982;
    L_p = -8.402;
    L_r = 2.193;
    L_da = 28.984;
    L_dr = 2.548;
    N_beta = 4.495;
    N_p = -0.3498;
    N_r = -0.7605;
    N_da = -0.2218;
    N_dr = -4.597;

    % Computing non-dimensional derivatives
    C_X_alpha = m*u0/(S*qinf) * X_w;
    C_X_u = m*u0/(S*qinf) * X_u;
    C_Z_u = m*u0/(S*qinf) * Z_u;
    C_Z_alpha = m*u0/(S*qinf) * Z_w;
    C_Y_beta = m*u0/(S*qinf) * Y_v;

    C_X_de = m/(S*qinf) * X_de;
    C_Z_de = m/(S*qinf) * Z_de;
    C_Y_da = m/(S*qinf) * Y_da;
    C_Y_dr = m/(S*qinf) * Y_dr;

    C_M_alpha = Iy*u0/(S*qinf*c) * M_w;
    C_M_u = Iy*u0/(S*qinf*c) * M_u;

    C_M_alphadot = 2*Iy*u0^2/(S*qinf*c^2) * M_wdot;

    C_M_q = 2*Iy*u0/(S*qinf*c^2) * M_q;
    C_L_p = 2*Ix*u0/(S*qinf*b^2) * L_p;
    C_L_r = 2*Ix*u0/(S*qinf*b^2) * L_r;
    C_N_p = 2*Iz*u0/(S*qinf*b^2) * N_p;
    C_N_r = 2*Iz*u0/(S*qinf*b^2) * N_r;

    C_M_de = Iy/(S*qinf*c) * M_de;
    C_L_beta = Ix/(S*qinf*b) * L_beta;
    C_L_da = Ix/(S*qinf*b) * L_da;
    C_L_dr = Ix/(S*qinf*b) * L_dr;
    C_N_beta = Iz/(S*qinf*b) * N_beta;
    C_N_da = Iz/(S*qinf*b) * N_da;
    C_N_dr = Iz/(S*qinf*b) * N_dr;

    %% Part b - converting to A, B matrix
    C_Y_p = 0; % Did not find, Not in NASA_CR-96008
    C_Y_r = 0; % Did not find, Not in NASA_CR-96008
    C_Y_phi = 0; % Did not find, Not in NASA_CR-96008
%     C_w = -weight/(S*qinf);
% 
%     X_dt = 1/m; % Corrected after solutions
%     C_X_dT = m/(S*qinf) * X_dt;  % Corrected after solutions
% 
%     C_X_alphadot = 0; % neglect according to Blakelock table 1-1
%     C_Z_alphadot = 0; % 0 from report
% 
%     C_X_q = 0; % neglect according to Blakelock table 1-1
%     Theta = 0;
%     C_Z_q = 0; % did not see a number for this

    % States
    % x = [beta, p, r, phi, psi]^T   u = [da, dr]^T
    % xdot = Ax + Bu
    % M1*xdot = M2*x + M3*u
    M1 = [m*u0/(S*qinf) 0                                   0                   0       0;
          0             Ix/(S*qinf*b)                       -Jxz/(S*qinf*b)     0       0;
          0             -Jxz/(S*qinf*b)                     Iz/(S*qinf*b)       0       0;
          0             0                                   0                   1       0;
          0             0                                   0                   0       1];


    M2 = [C_Y_beta  b/(2*u0)*C_Y_p  -(m*u0/(S*qinf)-b/(2*u0)*C_Y_r)     C_Y_phi             0;
          C_L_beta  b/(2*u0)*C_L_p  b/(2*u0)*C_L_r                      0                   0;
          C_N_beta  b/(2*u0)*C_N_p  b/(2*u0)*C_N_r                      0                   0;
          0         1               0                                   0                   0;
          0         0               1                                   0                   0];


    M3 = [C_Y_da    C_Y_dr;
          C_L_da    C_L_dr;
          C_N_da    C_N_dr;
          0         0;
          0         0];

    A = M1\M2;
    B = M1\M3;
end