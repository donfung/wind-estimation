function C = getMeasurementJacobian(mu)
    vg = mu(3);
    chi = mu(4);
        
    C = [1, 0, 0, 0, 0, 0;
         0, 1, 0, 0, 0, 0;
         0, 0, 1, 0, 0, 0;
         0, 0, 0, 1, 0, 0;
         0, 0, -cos(chi), vg*sin(chi), 1, 0;
         0, 0, -sin(chi), -vg*cos(chi), 0, 1];
end