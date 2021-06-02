function A = getStateTransitionDynamics(mu, uKF, dt)
    % Estimated states:
    vg  = mu(3);
    chi = mu(4);
    wnStc  = mu(5);
    weStc  = mu(6);
    wnTurb = mu(7);
    weTurb = mu(8);
    
    % Assumed known:
    va = uKF(1);
    q = uKF(2);
    r = uKF(3);
    phi = uKF(4);
    theta = uKF(5);
    psi = uKF(6);
    alt = uFK(7);
    delVa_delUs = uKF(8);  % Equation 29 
    delVa_delVs = uKF(9);

    %     delVa_delUs=(R_bn*[1 0 0]')'*v_aircraft_airspeed_body/v_airspeed_body_norm; % Equation 29
    %       delVa_delVs=(R_bn*[0 1 0]')'*v_aircraft_airspeed_body/v_airspeed_body_norm;  % Equation 29
    % Do DCM outside of the function and assign to R_bn

    psiDot = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);
    vgDot = ((va*cos(psi)+wnStc)*(-va*psiDot*sin(psi)) + (va*sin(psi) + weStc)*(va*psiDot*cos(psi)))/vg;

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
    A(1:6, 1:6) = 1 + A(1:6, 1:6);
    
 %% Linearized Dryden Model     
    u_t_ned=wnTurb;
    v_t_ned=weTurb;
    
    kts2mps = 0.5144;
    
    [Lu, Lv, Lw] = getTurbulentLengthScales(alt);
    [sigmaU, sigmaV, sigmaW] = getTurbulenceSigmas();
    
    
    u_t_ned=(1-airspeed * dt / Lu)*u_t_ned+sqrt(2 * airspeed * dt / L_u) * sigmaU * noise;
    u_t_ned = u_t_ned * kts2mps; % ug in m/s

    v_t_ned=(1 - airspeed * dt / Lv)*v_t_ned+sqrt(2*airspeed/Lu)*sigmaV*noise;
    v_t_ned=v_t_ned*0.5144; % vg in m/s
      
    %% partial derivatives of the dryden turbulence equation with respect to static wind velocity components
    a=(dt/Lu)*(delVa_delUs)*u_t_ned; 
    b=(dt/Lu)*(delVa_delVs)*u_t_ned ;
    c=(dt/Lv)*(delVa_delUs)*v_t_ned;
    d=(dt/Lv)*(delVa_delVs)*v_t_ned;
    %% partial derivatives of the dryden turbulence equation with respect to turbulent wind velocity components
    e= dt*(v_airspeed_body_norm/L_u);
    f= dt*(v_airspeed_body_norm/L_v);

    A_with_wind = [0, 0, cos(chi),    -vg*sin(chi),             0,      0 ,0,  0;
      0, 0, sin(chi),    vg*cos(chi),    0,                   0,      0 ,    0;
      0, 0, -vgDot/vg,   0,              -psiDot*va*sin(psi), psiDot*va*cos(psi), 0, 0;
      0, 0, dchiDotdvg,  dchiDotdchi,    0,                   0 ,      0,    0 ;
      0, 0, 0,           0,              0,                   0,       0 ,   0;
      0, 0, 0,           0,              0,                   0,       0,    0;
      0, 0 ,0,           0,              a,                   b,       e,    0;
      0, 0, 0,           0,              c,                   d,       0,    f;
      ];

end
