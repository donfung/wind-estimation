function var = getSensorVariances(sensorType)
% This function provides a struct containing measurement variances for
% aircraft sensors. 
%
% List of sensors:
%   1. Airspeed (Single-hole pitot tube)
%   2. IMU (Attitude and/or attitude rates)
%   3. GPS (var.position)
%   4. 

deg2rad = pi/180;

    switch sensorType
        case 'low' 
            % Low-tier (RC)
            var.airspeed_mps        = 1.5^2;              % SDP3x (https://www.sensirion.com/en/flow-sensors/differential-pressure-sensors/worlds-smallest-differential-pressure-sensor/)
            var.attitude_rad        = 0.5 * deg2rad;    % VectorNav-100 (Heading accuracy is 2deg RMS)
            var.pqr_radps           = 0.25^2 * deg2rad;  % VectorNav-100
            var.velocityGps_mps     = 0.1^2;       
            var.position_m          = 5^2;              % u-blox 4 (https://wiki.openstreetmap.org/wiki/GPS_Chipset)
            var.altitude_m          = 0.2; % 20cm accuracy
            
        case 'mid' 
            % Mid-tier (General Aviation)
            var.airspeed_mps        = 1^2;      % AeroProbe (http://www.aeroprobe.com/pitot-static-probe/#1492294367387-409b9ea2-0e4b)
            var.attitude_rad        = 0.15^2 * deg2rad;   % Honeywell i300 IMU (https://aerospace.honeywell.com/en/learn/products/sensors/hguide-i300)
            var.pqr_radps           = 0.15^2 * deg2rad;   % Honeywell i300 IMU
            var.velocityGps_mps     = 0.05^2;   % u-blox 8 (https://www.u-blox.com/sites/default/files/NEO-M8P_DataSheet_%28UBX-15016656%29.pdf)
            var.position_m          = 2.5^2;    % u-blox 8 (https://wiki.openstreetmap.org/wiki/GPS_Chipset)
            var.altitude_m          = 0.1;      % 10cm accuracy
        
        case 'high'
            % High tier(Fighter jets, airliners)
            var.airspeed_mps        = 0.0002^2;         % 0.02% of measurement Honeywell AADC (https://aerospace.honeywell.com/content/dam/aero/en-us/documents/learn/products/sensors/brochures/C61-0308-000-001-AirDataComputer-bro.pdf?download=true)
            var.attitude_rad        = 0.002^2 * deg2rad;          % Honeywell HG9900 (https://aerospace.honeywell.com/en/learn/products/sensors/hg9900-inertial-measurement-unit)
            var.pqr_radps           = 0.002^2 * deg2rad;          % Honewell HG9900
            var.velocityGps_mps     = (0.05/2.5) * 0.025 ^2; % Assume linearly scaled 
            var.position_m          = 0.025^2;          % Assume RTK accuracy of u-blox 8 (https://www.u-blox.com/sites/default/files/NEO-M8P_DataSheet_%28UBX-15016656%29.pdf) 
            var.altitude_m          = 0.01;      % 1cm accuracy

        otherwise
            % Default to low tier sensor suite
            var = getSensorVariances('low');
    end
    
end