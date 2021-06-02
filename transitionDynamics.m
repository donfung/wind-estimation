function mu_kp = transitionDynamics(mu, uKF, dt)
    vg     = mu(3);
    chi    = mu(4);
    wnStc  = mu(5);
    weStc  = mu(6);
    wnTurb = mu(7);
    weTurb = mu(8);
    
    va      = uKF(1);
    q       = uKF(2);
    r       = uKF(3);
    phi     = uKF(4);
    theta   = uKF(5);
    psi     = uKF(6);
    alt     = uKF(7);
    wnNoise = uKF(13);
    weNoise = uKF(14);
    
    psiDot = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);
    g = 9.81;
    
    % Get Turbulent properties
    [Lu, Lv, Lw]             = getTurbulentLengthScales(alt);
    [sigmaU, sigmaV, sigmaW] = getTurbulenceSigmas(alt);
    
    % Compute next turbulence value using difference equation (in MATLAB)
    u_t_ned = (1 - va * dt / Lu) * wnTurb + realsqrt(2 * va * dt / Lu) * sigmaU * wnNoise;
    v_t_ned = (1 - va * dt / Lv) * weTurb + realsqrt(2 * va * dt / Lv) * sigmaV * weNoise;

    % Convert from kts to mps
    kts2mps = 0.5144;
    u_t_ned = u_t_ned * kts2mps; 
    v_t_ned = v_t_ned * kts2mps; 
    
    mu_kp = [mu(1) + vg*cos(chi);
             mu(2) + vg*sin(chi);
             mu(3) + ((va*cos(psi)+wnStc)*(-va*psiDot*sin(psi)) + (va*sin(psi) + weStc)*(va*psiDot*cos(psi)))/vg;
             mu(4) + g/vg*tan(phi)*cos(chi - psi);
             mu(5);
             mu(6);
             u_t_ned;
             v_t_ned];
    
end
