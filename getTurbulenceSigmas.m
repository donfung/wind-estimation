function [sigmaU_mps, sigmaV_mps, sigmaW_mps] = getTurbulenceSigmas(alt_ft)
% Reference: https://www.mathworks.com/help/aeroblks/drydenwindturbulencemodeldiscrete.html
    
% The turbulence intensities are given below, where W20 is the wind speed at 20 feet (6 m).
% Typically for light turbulence, the wind speed at 20 feet is 15 knots; for moderate turbulence, 
% the wind speed is 30 knots, and for severe turbulence, the wind speed is 45 knots.
    kts2mps = 0.514;
    
    W20 = 25;  % knots wind speed at 20 feet, explained in MATLAB documentation
    
    %% noise amplitudes Equation 16 and 17
    sigmaU_kts = (0.1 * W20) / (0.177 + 0.000823 * alt_ft)^0.4;
    sigmaV_kts = sigmaU_kts;
    sigmaW_kts = 0.1 * W20; 

    sigmaU_mps = sigmaU_kts * kts2mps;
    sigmaV_mps = sigmaV_kts * kts2mps;
    sigmaW_mps = sigmaW_kts * kts2mps;
    
end

