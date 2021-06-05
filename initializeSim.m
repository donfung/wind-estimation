dt = 0.01;
mass = 1000;
J = diag([1, 1, 1]);

nStates = 18;
nEstStates = 8;
%% INITIAL CONDITIONS
alt0 = 600;
initVelBody  = [80, 43, 0]; % [u, v, w]_body 
initAttitude = [0, 0, 0]; % [phi, theta, psi]
initGroundSpeedEst = norm(initVelBody);

% Notes:
%   Heading:
%       Psi = 0: Aircraft points North
%       Psi = 90: East
%       Psi = 180: South

%% WIND PARAMETERS
windNorthStatic = 14;
windEastStatic = -9;
windDownStatic = 0;

%% FLAGS
% 1 = enable. 0 = disable
sensorNoiseFlag     = 1;  % Sensor noises
turbulentWindFlag   = 1;  % Turn wind turbulence on/off 
staticWindFlag      = 1;  % Turn static wind on/off
gustWindFlag        = 1;  % Turn gust (step changes) on/off

%% SENSORS
suite = 'high';  % String: 'low', 'mid', or 'high'
var        = getSensorVariances(suite);
sampleTime = getSensorSampleTimes(suite, dt);

%% EKF PARAMETERS
% mu = [pn, pe, vg, chi, wn, we]

Qekf = 1e-6*eye(nEstStates);
Qekf = diag([1e-5, 1e-5, ...   % pn, pe
             1, 1e-3, ...      % vg, chi
             1, 1, ...   % wn, we static
             1, 1]);         % wn, we turb
         
Rekf = 0.01*eye(7);
Rekf = diag([1, 1, ...         % pn, pe
             1, 1e-3, ...      % vg, chi
             1, 1]);     % wn, we
             
initEstimateEkf = [0, 0, ...
                   initGroundSpeedEst, initAttitude(3), ...
                   windNorthStatic + 4*randn, windEastStatic + 4*randn, ...
                   0, 0];
initCovarianceEkf = diag([1e-3, 1e-3, ...   % pn, pe
                          2, 5e-3, ...      % vg, chi
                          1e-4, 1e-4, ...   % wn, we static
                          10, 10]);         % wn, we turb


%% UKF
% Qukf = 0.01*eye(nEstStates);
% Rukf = 0.1*eye(6);
Qukf = diag([1e-5, 1e-5, ...   % pn, pe
             1, 1e-3, ...      % vg, chi
             .1, .1, ...   % wn, we static
             .1, .1]);         % wn, we turb
Rukf = diag([1, 1, ...         % pn, pe
             1, 1e-3, ...      % vg, chi
             1, 1]);     % wn, we

initEstimateUKF = [0, 0, initGroundSpeedEst, initAttitude(3),...
                   windNorthStatic + 1*randn, windEastStatic + 1*randn, ...
                   0, 0];
% initCovarianceUKF = 0.001*eye(nEstStates);
initCovarianceUKF = diag([1e-3, 1e-3, ...   % pn, pe
                          2, 5e-3, ...      % vg, chi
                          1e-4, 1e-4, ...   % wn, we static
                          1, 1]);         % wn, we turb

%% Particle filter
% num_particles = 100;
Qpf = 0.01*eye(6);
Rpf = 0.1*eye(6);

initEstimateParticleFilter = [0, 0, initGroundSpeedEst, initAttitude(3), 15, -32];
initCovarianceParticleFilter = 0.11*eye(6);

%% DISPLAY FOR USER 
disp(' ');
if sensorNoiseFlag 
    disp('Sensor noise enabled');
else
    disp('Sensor noise disabled');
end
if turbulentWindFlag 
    disp('Turbulent wind enabled');
else
    disp('Turbulent wind disabled');
end
if staticWindFlag 
    disp('Static wind enabled');
else
    disp('Static wind disabled');
end
if staticWindFlag 
    disp('Gusts enabled');
else
    disp('Gusts disabled');
end

disp(' ');
disp(['Static North Wind: ', num2str(windNorthStatic), 'mps']);
disp(['Static East Wind: ', num2str(windEastStatic), 'mps']);
