function A = getDynamicsJacobian(mu, uKF)
    % Estimated states:
    vg  = mu(3);
    chi = mu(4);
    wn  = mu(5);
    we  = mu(6);
    wtn = mu(7);
    wte = mu(8);
    % Assumed known:
    va = uKF(1);
    q = uKF(2);
    r = uKF(3);
    phi = uKF(4);
    theta = uKF(5);
    psi = uKF(6);
    alt=uFK(7);
    v_aircraft_ned=uKF(8:10);
    % Do DCM outside of the function and assign to R_bn
    
    
    
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
      

 %% Linearized Dryden Model     
       
       Q=2;
       noise=randn(1,1)*Q;
       v_aircraft_airspeed_body=R_bn*(v_aircraft_ned)-R_bn*v_wind_ned(1:3);% airspeed body frame

       v_airspeed_body_norm=norm(v_aircraft_airspeed_body);

       v_airspeed_body_norm_knots=v_airspeed_body_norm*1.943;
       alt_feet=alt*3.28;
       dt=0.1;
       u_t_ned=wtn;
       v_t_ned=wte;
       L_w=alt_feet;
       L_u=alt_feet/(0.177+0.000823*alt_feet)^1.2; %turbulence length scale
       L_v=L_u;
       
  

       u_t_ned=(1-v_airspeed_body_norm_knots*dt/L_u)*u_t_ned+sqrt(2*v_airspeed_body_norm_knots*dt/L_u)*sigma_u*noise;
       u_t_ned=u_t_ned*0.5144; % ug in m/s

       v_t_ned=(1-v_airspeed_body_norm_knots*dt/L_v)*v_t_ned+sqrt(2*v_airspeed_body_norm_knots*dt/L_u)*sigma_v*noise;
       v_t_ned=v_t_ned*0.5144; % vg in m/s

           
      
      delVa_delUs=(R_bn*[1 0 0]')'*v_aircraft_airspeed_body/v_airspeed_body_norm; % Equation 29

      delVa_delVs=(R_bn*[0 1 0]')'*v_aircraft_airspeed_body/v_airspeed_body_norm;  % Equation 29
      
     %% partial derivatives of the dryden turbulence equation with respect to static wind velocity components
      a=(dt/L_u)*(delVa_delUs)*u_t_ned; 
      b=(dt/L_u)*(delVa_delVs)*u_t_ned ;
      c=(dt/L_v)*(delVa_delUs)*v_t_ned;
      d=(dt/L_v)*(delVa_delVs)*v_t_ned;
      %% partial derivatives of the dryden turbulence equation with respect to turbulent wind velocity components
      e= 1-dt*(v_airspeed_body_norm/L_u);
      f= 1-dt*(v_airspeed_body_norm/L_v);
      
     
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
