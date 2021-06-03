function f = dynamics(mu, uKF, dt)
    vg  = mu(3);
    chi = mu(4);
    wn  = mu(5);
    we  = mu(6);
    wnt = mu(7);
    wet = mu(8);
    
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
            ((va*cos(psi)+wn + wnt)*(-va*psiDot*sin(psi)) + (va*sin(psi) + we + wet)*(va*psiDot*cos(psi)))/vg;
            g/vg*tan(phi)*cos(chi - psi);
            0;
            0;
            0;  % ??? (north turb)
            0]; % ??? (east turb
        
    f = mu + dt*xdot;
    
end
