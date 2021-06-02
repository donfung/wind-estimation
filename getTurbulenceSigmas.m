function [sigma_w,sigma_v,sigma_u] = getTurbulenceSigmas(alt_feet)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
     W_20=25;%knots wind speed at 20 feet
     %% noise amplitudes Equation 16 and 17
       sigma_w=0.1*W_20; 
       sigma_v=(sigma_w*0.1)/(0.177+0.000823*alt_feet)^0.4;
       sigma_u=sigma_v;
end

