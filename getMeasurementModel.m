function h = getMeasurementModel(mu, uKF)
    % Estimated states: [pn, pe, vg, chi, wn, we]
    pn = mu(1);
    pe = mu(2);
    vg = mu(3);
    chi = mu(4);
    wn = mu(5);
    we = mu(6);
    
    % Airspeed and Heading assumed known
    va = uKF(1);
    psi = uKF(6);
    
    h = [pn;
         pe;
         vg;
         chi;
         va*cos(psi) - vg*cos(chi) + wn;
         va*sin(psi) - vg*sin(chi) + we];

end
