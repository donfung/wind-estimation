function g = measurementFcn()
    K = 1;
    Va = 1;
    CL0 = 1;
    CLalpha = 1;
    alpha = 1;
    vel_s = [1; 1; 1];
    vel_t = [1; 1; 1];
    u_r_meas = 1;
    Rn2b = eye(3);
    d1 = [1, 0, 0];
    g = [-K*Va^2*(CL0 + CLalpha*alpha);
         d1*Rn2b*(vel_s + vel_t) + u_r_meas];
end

function C = measurementJacobian()

    % h1 = z-acceleration
    % h2 = u body velocity
    
    K = 1;
    Va = 1;
    d1 = [1; 0; 0];
    d2 = [0; 1; 0];
    d3 = [0; 0; 1];
    
    vr_bar = [1; 1; 1];  % Definition eqn (20) Wenz
    CL0 = 1;
    CLalpha = 1;
    alpha = 1;
    ur = 1;
    wr = 1;
    
    Rn2b = eye(3);
    
    
    dh1_dus = 2*K*transpose(Rn2b*d1)*vr_bar*(CL0 + CLalpha*alpha) + (-Rn2b(3,1)*ur + Rn2b(1,1)*wr)/(ur^2 + wr^2);   % Eqn (38) Wenz
    dh1_dvs = 2*K*transpose(Rn2b*d2)*vr_bar*(CL0 + CLalpha*alpha) + (-Rn2b(3,2)*ur + Rn2b(1,2)*wr)/(ur^2 + wr^2);   % Eqn (39) Wenz     
    dh1_dws = 2*K*transpose(Rn2b*d3)*vr_bar*(CL0 + CLalpha*alpha) + (-Rn2b(3,3)*ur + Rn2b(1,3)*wr)/(ur^2 + wr^2);   % Eqn (40) Wenz
    
    dh1_dut = K*Va^2*CLalpha*(-Rn2b(3,1)*ur - Rn2b(1,1)*wr)/(ur^2 + wr^2);
    dh1_dvt = K*Va^2*CLalpha*(-Rn2b(3,2)*ur - Rn2b(1,2)*wr)/(ur^2 + wr^2);
    dh1_dwt = K*Va^2*CLalpha*(-Rn2b(3,3)*ur - Rn2b(1,3)*wr)/(ur^2 + wr^2);
    
    dh2_dVs = transpose(d1) * Rn2b;
    
    dh2_dVt = transpose(d1) * Rn2b;
    
    C = [dh1_dus, dh1_dvs, dh1_dws, dh1_dut, dh1_dvt, dh1_dwt;
         dh2_dVs, dh2_dVt];
end