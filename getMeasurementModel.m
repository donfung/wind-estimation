function h = getMeasurementModel(mu, uKF)
    % Estimated states: [pn, pe, vg, chi, wn, we]
    pn = mu(1);
    pe = mu(2);
    vg = mu(3);
    chi = mu(4);
    wnStatic = mu(5);
    weStatic = mu(6);
    wnTurb   = mu(7);
    weTurb   = mu(8);
    
    % Airspeed and Heading assumed known
    va = uKF(1);
    psi = uKF(6);
    d1Rn2b = uKF(10:12);
    
    vs = [wnStatic; weStatic; 0];
    vt = [wnTurb; weTurb; 0];
    
    h = [pn;
         pe;
         vg;
         chi;
         va*cos(psi) - vg*cos(chi) + wnStatic;
         va*sin(psi) - vg*sin(chi) + weStatic;
         d1Rn2b*(vs + vt) + va];

end
