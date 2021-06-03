function A = getDynamicsJacobian(mu, uKF, dt)
    % Estimated states:
    vg  = mu(3);
    chi = mu(4);
    wn  = mu(5);
    we  = mu(6);
    wnTurb = mu(7);
    weTurb = mu(8);
    
    % Assumed known:
    va      = uKF(1);
    q       = uKF(2);
    r       = uKF(3);
    phi     = uKF(4);
    theta   = uKF(5);
    psi     = uKF(6);
    alt     = uKF(7);
    delVa_delUs = uKF(8);  % Equation 29 
    delVa_delVs = uKF(9);
    wnNoise     = uKF(13);
    weNoise     = uKF(14);
    
    
    psiDot = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);
    vgDot = ((va*cos(psi)+wn)*(-va*psiDot*sin(psi)) + (va*sin(psi) + we)*(va*psiDot*cos(psi)))/vg;
    
    g = 9.81;
    dchiDotdvg  = -g/vg^2*tan(phi)*cos(chi - psi);
    dchiDotdchi = -g/vg*tan(phi)*sin(chi - psi);

    A = zeros(8);
    
     A(1:6, 1:6) = [0, 0, cos(chi),    -vg*sin(chi),   0,                   0;
          0, 0, sin(chi),    vg*cos(chi),    0,                   0;
          0, 0, -vgDot/vg,   0,              -psiDot*va*sin(psi), psiDot*va*cos(psi);
          0, 0, dchiDotdvg,  dchiDotdchi,    0,                   0;
          0, 0, 0,           0,              0,                   0;
          0, 0, 0,           0,              0,                   0];
    
 %% Linearized Dryden Model (MIL-F-8785C)   
    [Lu, Lv, Lw]                = getTurbulentLengthScales(alt);        % Altitude in meters
    [sigmaU, sigmaV, sigmaW]    = getTurbulenceSigmas(alt);
    
    %% Partial derivatives of the dryden turbulence transition equation with respect to static and turbulent wind velocity components
%     A(7, 5) = -1 * (dt/Lu) * (delVa_delUs) * wnTurb;
%     A(7, 6) = -1 * (dt/Lu) * (delVa_delVs) * wnTurb;
%     A(8, 5) = -1 * (dt/Lv) * (delVa_delUs) * weTurb;
%     A(8, 6) = -1 * (dt/Lv) * (delVa_delVs) * weTurb;
%     A(7, 7) = -1 * dt*(va / Lu);
%     A(8, 8) = -1 * dt*(va / Lv);

% Playing around with different derivatives
%     A(7, 7) = -va/Lu*wnTurb + 1/dt * sqrt(2 * va * dt / (Lu))*sigmaU;
%     A(8, 8) = -va/Lv*weTurb + 1/dt * sqrt(2 * va * dt /Lv)*sigmaV;

% Playing around with different derivatives
%     A(7, 7) = -va/Lu*wnTurb + sqrt(2 * va  / Lu)*sigmaU;
%     A(8, 8) = -va/Lv*weTurb + sqrt(2 * va  / Lv)*sigmaV;

end
