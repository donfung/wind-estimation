function A = getDynamicsJacobian(mu, uKF)
    % Estimated states:
    vg  = mu(3);
    chi = mu(4);
    wn  = mu(5);
    we  = mu(6);
    
    % Assumed known:
    va = uKF(1);
    q = uKF(2);
    r = uKF(3);
    phi = uKF(4);
    theta = uKF(5);
    psi = uKF(6);
    
    psiDot = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);
    vgDot = ((va*cos(psi)+wn)*(-va*psiDot*sin(psi)) + (va*sin(psi) + we)*(va*psiDot*cos(psi)))/vg;
    
    g = 9.81;
    dchiDotdvg  = -g/vg^2*tan(phi)*cos(chi - psi);
    dchiDotdchi = -g/vg*tan(phi)*sin(chi - psi);

    
     A = [0, 0, cos(chi),    -vg*sin(chi),   0,                   0;
          0, 0, sin(chi),    vg*cos(chi),    0,                   0;
          0, 0, -vgDot/vg,   0,              -psiDot*va*sin(psi), psiDot*va*cos(psi);
          0, 0, dchiDotdvg,  dchiDotdchi,    0,                   0;
          0, 0, 0,           0,              0,                   0;
          0, 0, 0,           0,              0,                   0];

end
