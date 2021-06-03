function C = getMeasurementJacobianTest(mu, uKF)
    vg = mu(3);
    chi = mu(4);
    
    d1Rn2b = uKF(10:12);
    
    C = [1, 0, 0, 0, 0, 0, 0, 0;    
         0, 1, 0, 0, 0, 0, 0, 0;
         0, 0, 1, 0, 0, 0, 0, 0;
         0, 0, 0, 1, 0, 0, 0, 0;
         0, 0, -cos(chi),  vg*sin(chi), 1, 0, 1, 0;         % wn static
         0, 0, -sin(chi), -vg*cos(chi), 0, 1, 0, 1];         % we static

end