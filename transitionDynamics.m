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
    
    kts2mps = 0.514;
    mps2kts = kts2mps^-1;
    va_kts = va * mps2kts;
    
    m2ft = 3.28;
    Lu_ft = Lu * m2ft;
    Lv_ft = Lv * m2ft;
    
    sigmaU_kts = sigmaU * mps2kts;
    sigmaV_kts = sigmaV * mps2kts;
    % Compute next turbulence value using difference equation (in MATLAB)
    u_t_ned_kts = (1 - va_kts * dt / Lu_ft) * wnTurb + realsqrt(2 * va_kts * dt / Lu_ft) * sigmaU_kts * wnNoise;
    v_t_ned_kts = (1 - va_kts * dt / Lv_ft) * weTurb + realsqrt(2 * va_kts * dt / Lv_ft) * sigmaV_kts * weNoise;

    % Convert from kts to mps
    kts2mps = 0.5144;
    u_t_ned_mps = u_t_ned_kts * kts2mps; 
    v_t_ned_mps = v_t_ned_kts * kts2mps; 
    
    mu_kp = [mu(1) + dt * vg*cos(chi);
             mu(2) + dt * vg*sin(chi);
             mu(3) + dt * ((va*cos(psi)+wnStc)*(-va*psiDot*sin(psi)) + (va*sin(psi) + weStc)*(va*psiDot*cos(psi)))/vg;
             mu(4) + dt * g/vg*tan(phi)*cos(chi - psi);
             mu(5);
             mu(6);
             u_t_ned_mps;
             v_t_ned_mps];
    
end
