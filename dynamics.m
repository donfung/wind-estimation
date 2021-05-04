% F16 Linear Aircraft Dynamics
%
% X = [Va, alpha, theta, q, pow, beta, phi, p, r] 
% where:
%       Va = airspeed
%    alpha = angle of attack
%      pow = engine power state **I don't know what this is**
%     beta = sideslip angle
%
% u = [dt, de, da, dr]

A = [-0.127,    -235, -32.2,   -9.51, 0.314, -0.0028, 0.00126,   5e-5,  2e-4;
     -7e-4,   -0.969,     0,   0.908, -2e-4,  1.5e-5,       0,  -4e-5, -1e-5;
         0,        0,     0,       1,     0,       0,       0,      0,     0;
      1e-8,     2e-5,  3e-6,    8e-7, -3e-8,  -0.322,  0.0612,  0.298, -0.948;
         0,        0,     0,       1,     0,       0,   0.093,      1,   0.31;
     -3e-7, -0.00248,     0,    3e-4,     0,   -62.5,       0,     -3,   1.99;
     -3e-6, -0.00188,     0, 0.00254,     0,    7.67,       0, -2.622, -0.629];
 
 B = [0,   -0.244,   6e-6,    2e-5;
      0, -0.00209,      0,       0;
      0,        0,      0,       0;
      0,   -0.199,      0,       0;
      1087,     0,      0,       0;
      0,     2e-8,   3e-4,    8e-4;
      0,        0,      0,       0;
      0,        0, -0.645,   0.126;
      0,        0, -0.018, -0.0657];

% We will change the C matrix. Leaving here for reference for now
C = [0.0208, 15.2, 0, 1.45, 0, -4.5e-4, 0, 0, 0;    % a_n (normal acceleration)
          0,    0, 0,    1, 0,       0, 0, 0, 0;    % q
          0, 57.3, 0,    0, 0,       0, 0, 0, 0];   % alpha

% We'll probably use D = 0, but the book provided a D matrix. May need it
% somehow. Keeping here for reference.
D = [0, 0.0333, 0, 0;
     0,      0, 0, 0;
     0,      0, 0, 0];
 
 
    
          