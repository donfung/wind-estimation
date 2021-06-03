function [Lu, Lv, Lw] = getTurbulentLengthScales(alt_m)
    % The equations given by MATLAB use altitude in ft. Need to convert
    % our altitude in meters to ft
    %
    % Reference: https://www.mathworks.com/help/aeroblks/drydenwindturbulencemodeldiscrete.html
    
    m2ft = 3.28;
    ft2m = 1/m2ft;
    alt_ft = alt_m * m2ft;
    
    % MIL-F-8785C Dryden Model: 
    % if alt_ft <= 2000  
    % "Low altitude model"
    Lu_ft = alt_ft / (0.177+0.000823 * alt_ft)^1.2; 
    Lv_ft = Lu_ft;
    Lw_ft = alt_ft;
    
    if alt_ft > 2000 
        % "Medium/High altitude model"
        Lu_ft = 1750;
        Lv_ft = 1750;
        Lw_ft = 1750;
    end
    
    Lu = Lu_ft * ft2m;
    Lv = Lv_ft * ft2m;
    Lw = Lw_ft * ft2m;
    
end