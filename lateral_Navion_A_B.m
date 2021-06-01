%% Navion lateral dynamics

% States
% x = [beta, p, r, phi, psi]^T   u = [da, dr]^T 
% x = normalized v vel (beta), roll rate p, yaw rate r, roll angle phi, yaw angle psi
% u = aileron deflection, rudder deflection
% xdot = Ax + Bu

function [A,B] = lateral_Navion_A_B()
    % Aircraft geometrical parameters
    m = 85.4; % slugs
    u0 = 176; % ft/s 
    S = 180; % ft^2
    qinf = 36.8; % lb/ft^2
    
    Ix = 1048; % slug-ft^2
    Iy = 3000; % slug-ft^2
    Iz = 3530; % slug-ft^2
    Jxz = 0; % slug-ft^2
    
    c = 5.7; % ft
    b = 33.4; % ft
    
    C_Y_beta = -0.564;
    C_Y_p = 0; % neglected % Table 3-1 
    C_Y_r = 0; % neglected % Table 3-1 
    C_Y_phi = 0.41;% mg/Sq = CL from page 122 of Blakelock; % Table 3-1 assuming theta = 0
    C_Y_psi = 0; % from page 122 of Blakelock % Table 3-1 assuming theta = 1
    C_Y_da = 0.0;
    C_Y_dr = 0.157;
    
    C_L_beta = -0.074;
    C_L_p= -0.410;
    C_L_r = 0.107;
    C_L_da = 0.1342;
    C_L_dr = 0.0118;
    
    C_N_beta = 0.0701;
    C_N_p = -0.0575;
    C_N_r = -0.125;
    C_N_da = -0.00346;
    C_N_dr = -0.0717;

    % Putting nondim derivatives in A, B matrix
    % States
    % x = [beta, p, r, phi, psi]^T   u = [da, dr]^T
    % xdot = Ax + Bu
    % M1*xdot = M2*x + M3*u
    % Equation 3-17 Blakelock
    M1 = [m*u0/(S*qinf) 0                                   0                   0       m*u0/(S*qinf);
          0             Ix/(S*qinf*b)                       -Jxz/(S*qinf*b)     0       0;
          0             -Jxz/(S*qinf*b)                     Iz/(S*qinf*b)       0       0;
          0             0                                   0                   1       0;
          0             0                                   0                   0       1];


    M2 = [C_Y_beta  b/(2*u0)*C_Y_p  b/(2*u0)*C_Y_r                      C_Y_phi             C_Y_psi;
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