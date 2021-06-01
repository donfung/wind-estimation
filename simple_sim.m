close all;
clear all;
% set(0,'DefaultFigureWindowStyle','docked');  

plotTrueStates = 1;
plotEstimates = 1;

t = 200;
dt = 0.01;
time = 0:dt:t;
n = length(time);

% AIRCRAFT PARAMETERS 
Jx = 5; Jy = 1; Jz = 10; Jxz = 0.01;
J = diag([Jx, Jy, Jz]);
J(1, 3) = Jxz;
J(3, 1) = Jxz;
m = 500;

x = zeros(18, n);

% UVW is ground speed. i.e uvwGround = C_body2inertial*uvw_air + wind_in_NED
windNed0 = [-50, 77, 0];
uvw0 = [2, 7, 0];     % THIS ONE WORKS
% uvw0 = [50, 30, 0];   % THIS ONE DOESNT WORK. It seems like large values of u, v cause the estimator to fail 

vg0 = norm(uvw0);
va = uvw0 - windNed0;
va0 = norm(uvw0 - windNed0);
chi0 = atan2(uvw0(2), uvw0(1));
psi0 = atan2(va(2), va(1));
% psi0 = 0;
x(:, 1) = [0, 0, -1000, ...     % 1000m alt           (1-3)
           uvw0, ...            % 50 mps              (4-6)
           0, 0, psi0, ...      % level attitude      (7-9)
           0, 0, 0, ...         % zero body rotation  (10-12)
           vg0, ...             % ground speed        (13)
           chi0, ...            % course angle        (14)
           va0, ...             % airspeed            (15)
           windNed0]';          % wind NED            (16-18)
 

%% True states
for i = 2:n
    xyzNed  = x(1:3, i-1);
    uvw     = x(4:6, i-1);
    att     = x(7:9, i-1);
    pqr     = x(10:12, i-1);
    windNed = x(16:18, i-1);

    Rn2b        = getDcm(att(1), att(2), att(3));
    vg          = norm(uvw);
    x(13, i-1)  = vg;
    
    F   = [0;0;0];
    LMN = [0; 0; 0];

    xyzNedDot   = xyzNedDeriv(att, uvw);
    uvwDot      = uvwDeriv(uvw, pqr, F, m);
    attDot      = attDeriv(att, pqr);
    pqrDot      = pqrDeriv(pqr, J, LMN);
    vgDot       = vgDeriv(x(:, i-1));
    chiDot      = 0;    % Manually update course angle
    vaDot       = 0;    % Manually update Va
    windDot     = [0;0;0];
    xdot        = [xyzNedDot; uvwDot; attDot; pqrDot; vgDot; chiDot; vaDot; windDot];

    x(:, i) = x(:, i-1) + xdot*dt;
    
    x(14, i) = atan2(uvw(2), uvw(1));  % course angle Chi
    va = uvw - windNed;
%     x(9, i) = atan2(va(2), va(1));
    x(15, i) = norm(uvw - windNed);  % airspeed

%     y(:, i) = measModel(x(:, i));
    if i > 3000 && i < 6000
        windNed = [50, 12, 0];
        x(16:18, i) = windNed;
    elseif i > 6000 && i < 12000
        windNed = [-67, 88, 0];
        x(16:18, i) = windNed;
    elseif i > 12000 && i < 17000
        windNed = [0, -50, 0];
        x(16:18, i) = windNed;
    end
    
end

if (plotTrueStates)
    figure;
    plot(x(1, :), x(2, :));
    title('XY NED');

    figure;
    hold on;
    plot(time, x(4, :));
    plot(time, x(5, :));
    plot(time, x(6, :));
    legend('u', 'v', 'w');
    title('uvw Ground');
    
    figure;
    hold on;
    plot(time, 180/pi*x(9, :));
    plot(time, 180/pi*x(14, :));
    legend('Psi', 'Chi');
    title('Psi vs Chi');
end

%% ESTIMATE

R = 0.1*eye(6);

mu = zeros(6, n);
cov = zeros(6, 6, n);
mu(:, 1) = [0; 0; vg0; 0; 0; 0];  % pn, pe, vg, chi, wn, we
cov(:, :, 1) = 0.1*eye(6);
Q = 0.1*eye(6);


for k = 2:n
    u_k     = [x(15, k-1);      % Va
               x(11, k-1);      % q
               x(12, k-1);      % r
               x(7, k-1);       % phi
               x(8, k-1);      % theta
               x(9, k-1)];      % psi
    y_kp    = measModel(x(:, k-1));% + mvnrnd(zeros(6,1), R)';
    
    [mu_kp, Sig_kp] = extended_Kalman_filter(mu(:, k-1), cov(:, :, k-1), u_k, y_kp, Q, R, dt);
    
%     verbose = true;
%     [mu_kp, Sig_kp] = iEKF(mu(:, k-1), cov(:, :, k-1), u_k, y_kp, Q, R, dt, verbose);
    
    mu(:, k) = mu_kp;
    cov(:, :, k) = Sig_kp;
end

figure;
plot(mu(1, :), mu(2, :));
legend('Est N', 'Est E');
title('Estimated NE position');

figure;
subplot(2,1,1);
hold on;
plot(time, mu(5, :));
plot(time, x(16, :));
legend('Estimated', 'True');
title('Estimated North Wind');

subplot(2,1,2);
hold on;
plot(time, mu(6, :));
plot(time, x(17, :));
legend('Estimated', 'True');
title('Estimated East Wind');

function [mu_kp, Sig_kp] = extended_Kalman_filter(mu_k, Sig_k, u_k, y_kp, Q, R, dt)
    % Prediction step
    A = dynJacobian(mu_k, u_k);
    mu_pred = dynamics(mu_k, u_k, dt);
    Sig_pred = A*Sig_k*A' + Q;

    % Update step
    C = measJacobian(mu_pred, u_k);
    ybar = y_kp - measEkf(mu_pred, u_k);
    K = Sig_pred*C'/(C*Sig_pred*C' + R); % Kalman gain
    mu_kp = mu_pred + K*ybar;
    Sig_kp = Sig_pred - K*C*Sig_pred;
end

% iEKF
function [mu_k_k,Sig_k_k] = iEKF(mu_km_km, Sig_km_km, u_km, y_k, Q, R, dt, verbose)
    % predict step
    A_k = dynJacobian(mu_km_km, u_km);
    mu_k_km = dynamics(mu_km_km, u_km, dt);
    Sig_k_km = A_k*Sig_km_km*A_k' + Q;
    
    % update step
    mu_k_old = mu_k_km; i = 0;
    converged = false; 
    while ~converged
        ybar_k = y_k - measEkf(mu_k_old, u_km);
%         C_k = C(mu_k_old);
        C_k = measJacobian(mu_k_old, u_km);
        K_k = Sig_k_km * C_k' / (C_k*Sig_k_km*C_k'+R);
        mu_k_new = mu_k_km + K_k*ybar_k + K_k*C_k*(mu_k_old - mu_k_km);
        if (mu_k_new == mu_k_old)
            converged = true;
            if verbose
                fprintf('iEKF: Converged at iteration %i \n',i);
            end
        end
        if i>100
            converged = true;
            if verbose
                fprintf('iEKF: Timing out after 100 iterations\n');
            end
        end
        i = i+1;
        mu_k_old = mu_k_new;        
    end
    mu_k_k = mu_k_new;
    Sig_k_k = Sig_k_km - K_k*C_k*Sig_k_km;
end

function xyzNedDot = xyzNedDeriv(att, uvw)
    phi = att(1);
    theta = att(2);
    psi = att(3);
    
    Rn2b = getDcm(phi, theta, psi);

    xyzNedDot = Rn2b'*uvw;
end

function Rn2b = getDcm(phi, theta, psi)
    Rn2b = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
            cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
            -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];
end

function uvwDot = uvwDeriv(uvw, pqr, F, m)
    p = pqr(1);
    q = pqr(2);
    r = pqr(3);
    u = uvw(1);
    v = uvw(2);
    w = uvw(3);
    
    uvwDot = [r*v - q*w; p*w - r*u; q*u - p*v] + F/m;
end
   
function attDot = attDeriv(att, pqr)
    phi = att(1);
    theta = att(2);
    
    Rbody2att = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
                 0, cos(phi), -sin(phi);
                 0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

    attDot = Rbody2att * pqr;
end

function pqrDot = pqrDeriv(pqr, J, LMN)
    Jx = J(1,1);
    Jy = J(2,2);
    Jz = J(3,3);
    Jxz = J(1,3);
    
    L = LMN(1);
    M = LMN(2);
    N = LMN(3);
    
    p = pqr(1);
    q = pqr(2);
    r = pqr(3);
    
    gamma = Jx*Jz - Jxz^2;
    g1 = Jxz*(Jx-Jy+Jz)/gamma;
    g2 = Jz*(Jz-Jy)/gamma;
    g3 = Jz/gamma;
    g4 = Jxz/gamma;
    g5 = (Jz-Jx)/Jy;
    g6 = Jxz/Jy;
    g7 = (Jx-Jy)*Jx/gamma;
    g8 = Jx/gamma;

    pqrDot = [g1*p*q - g2*q*r; g5*p*r - g6*(p^2 - r^2); g7*p*q - g1*q*r] + [g3*L + g4*N; M/Jy; g4*L + g8*N];
end

function vgDot = vgDeriv(x)
    va = x(15);
    psi = x(9);
    wn = x(16);
    we = x(17);
    psiDot = psiDeriv(x);
    vg = x(13);
    
    vgDot = ((va*cos(psi)+wn)*(-va*psiDot*sin(psi)) + (va*sin(psi) + we)*(va*psiDot*cos(psi)))/vg;
end

function psiDot = psiDeriv(x)
    q = x(11);
    r = x(12);
    phi = x(7);
    theta = x(8);
    
    psiDot = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);
end

function f = dynamics(xKF, uKF, dt)
    vg  = xKF(3);
    chi = xKF(4);
    wn  = xKF(5);
    we  = xKF(6);
%     psi = xKF(7);
    
    va = uKF(1);
    q = uKF(2);
    r = uKF(3);
    phi = uKF(4);
    theta = uKF(5);
    psi = uKF(6);
    
    psiDot = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);
    g = 9.81;
    
    xdot = [vg*cos(chi);
            vg*sin(chi);
            ((va*cos(psi)+wn)*(-va*psiDot*sin(psi)) + (va*sin(psi) + we)*(va*psiDot*cos(psi)))/vg;
            g/vg*tan(phi)*cos(chi - psi);
            0;
            0];
%             q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta)];
        
    f = xKF + dt*xdot;
    
end

function A = dynJacobian(xKF, uKF)
    vg = xKF(3);
    chi = xKF(4);
    wn = xKF(5);
    we = xKF(6);
%     psi = xKF(7);
    
    va = uKF(1);
    q = uKF(2);
    r = uKF(3);
    phi = uKF(4);
    theta = uKF(5);
    psi = uKF(6);
    
    psiDot = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);
    vgDot = ((va*cos(psi)+wn)*(-va*psiDot*sin(psi)) + (va*sin(psi) + we)*(va*psiDot*cos(psi)))/vg;
    
    g = 9.81;
    dvgDotdpsi  = (-psiDot*va*(wn*cos(psi) + we*sin(psi)))/vg;
    dchiDotdvg  = -g/vg^2*tan(phi)*cos(chi - psi);
    dchiDotdchi = -g/vg*tan(phi)*sin(chi - psi);
    dchiDotdpsi = g/vg*tan(phi)*sin(chi - psi);

%     A = [0, 0, cos(chi),    -vg*sin(chi),   0, 0, 0;
%          0, 0, sin(chi),    vg*cos(chi),    0, 0, 0;
%          0, 0, -vgDot/vg,   0,              -psiDot*va*sin(psi), psiDot*va*cos(psi), dvgDotdpsi;
%          0, 0, dchiDotdvg,  dchiDotdchi,    0,                  0, dchiDotdpsi;
%          0, 0, 0, 0, 0, 0, 0;
%          0, 0, 0, 0, 0, 0, 0;
%          0, 0, 0, 0, 0, 0, 0];

         A = [0, 0, cos(chi),    -vg*sin(chi),   0, 0;
         0, 0, sin(chi),    vg*cos(chi),    0, 0;
         0, 0, -vgDot/vg,   0,              -psiDot*va*sin(psi), psiDot*va*cos(psi);
         0, 0, dchiDotdvg,  dchiDotdchi,    0,                  0;
         0, 0, 0, 0, 0, 0;
         0, 0, 0, 0, 0, 0];

end

function hEkf = measEkf(xKF, uKF)
    pn = xKF(1);
    pe = xKF(2);
    vg = xKF(3);
    chi = xKF(4);
    wn = xKF(5);
    we = xKF(6);
    
    va = uKF(1);
    psi = uKF(6);
    
    hEkf = [pn;
            pe;
            vg;
            chi;
            va*cos(psi) - vg*cos(chi) + wn;
            va*sin(psi) - vg*sin(chi) + we];

end

function h = measModel(x)
    pn = x(1);
    pe = x(2);
    psi = x(9);
    vg = x(13);
    chi = x(14);
    va = x(15);    
    wn = x(16);
    we = x(17);
    
    h = [pn;
         pe;
         vg;
         chi;
         va*cos(psi) - vg*cos(chi) + wn;
         va*sin(psi) - vg*sin(chi) + we];
     
end

function C = measJacobian(xKF, uKF)
    vg = xKF(3);
    chi = xKF(4);
%     psi = xKF(7);
    
    va = uKF(1);
    psi = uKF(6);
    
    C = [1, 0, 0, 0, 0, 0;
         0, 1, 0, 0, 0, 0;
         0, 0, 1, 0, 0, 0;
         0, 0, 0, 1, 0, 0;
         0, 0, -cos(chi), vg*sin(chi), 1, 0;
         0, 0, -sin(chi), -vg*cos(chi), 0, 1];
end