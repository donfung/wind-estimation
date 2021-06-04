function sampleTime = getSensorSampleTimes(sensorType, dt)

    switch sensorType
        case 'low' 
            % Low-tier (RC)
            sampleTime.airspeed        = 1/50;  % 50Hz
            sampleTime.attitude        = 1/400; % 400Hz
            sampleTime.pqr             = 1/400; % 400Hz
            sampleTime.gpsGroundSpeed  = 1/20;  % 20Hz    
            sampleTime.gpsPosition     = 1/10;  % 20Hz
            sampleTime.gpsCourseAngle  = 1/20;  % 20Hz
            sampleTime.altimeter       = 1/50;  % 50Hz
            
        case 'mid' 
            % Mid-tier (General Aviation)
            sampleTime.airspeed         = 1/100;      
            sampleTime.attitude         = 1/800;   
            sampleTime.pqr              = 1/800;
            sampleTime.gpsGroundSpeed   = 1/50;   
            sampleTime.gpsPosition      = 1/20;    
            sampleTime.gpsCourseAngle   = 1/50;
            sampleTime.altimeter        = 1/100;    
        
        case 'high'
            % High tier(Fighter jets, airliners)
            sampleTime.airspeed         = 1/800;
            sampleTime.attitude         = 1/6400;
            sampleTime.pqr              = 1/6400;
            sampleTime.gpsGroundSpeed   = 1/400;
            sampleTime.gpsCourseAngle   = 1/400;
            sampleTime.gpsPosition      = 1/400;
            sampleTime.altimeter        = 1/800;

        otherwise
            % Default to mid tier sensor suite
            sampleTime = getSensorVariances('mid');
    end
    
    fields = fieldnames(sampleTime);
    for i = 1:numel(fields)
        
        if sampleTime.(fields{i}) < dt
            % Can't have sample time to be less than Simulink's time step
            sampleTime.(fields{i}) = dt;
        end
    end
    
end