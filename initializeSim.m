dt = 0.01;
mass = 1000;
J = diag([1, 1, 1]);

nStates = 18;
nEstStates = 8;
%% INITIAL CONDITIONS
alt0 = 1000;
initVelBody  = [80, 43, 0]; % [u, v, w]_body 
initAttitude = [0, 0, 0]; % [phi, theta, psi]
initGroundSpeedEst = norm(initVelBody);

% Notes:
%   Heading:
%       Psi = 0: Aircraft points North
%       Psi = 90: East
%       Psi = 180: South

%% WIND PARAMETERS
% windNorthStatic = 15;
% windEastStatic = -32;
windNorthStatic = -32;
windEastStatic = -15;
windDownStatic = 0;

%% FLAGS
% 1 = enable. 0 = disable
sensorNoiseFlag     = 0;  % Sensor noises
turbulentWindFlag   = 0;  % Turn wind turbulence on/off 
staticWindFlag      = 1;  % Turn static wind on/off
gustWindFlag        = 1;  % Turn gust (step changes) on/off

%% SENSORS
suite = 'low';  % String: 'low', 'mid', or 'high'
var = getSensorVariances(suite);

%% EKF PARAMETERS
% mu = [pn, pe, vg, chi, wn, we]

% Working
% Qekf = 1*eye(6);
% Rekf = 1*eye(6);
% initEstimateEkf = [0, 0, initGroundSpeedEst, initAttitude(3), 0, 0];
% initCovarianceEkf = 0.1*eye(6);

% with turb
Qekf = 1e-4*eye(8);
Rekf = 1e-3*eye(7);
initEstimateEkf = [0, 0, initGroundSpeedEst, initAttitude(3), 0, 0, 0, 0];
initCovarianceEkf = 0.1*eye(8);

%% UKF
Qukf = 0.01*eye(6);
Rukf = 0.1*eye(6);

initEstimateUKF = [0, 0, initGroundSpeedEst, initAttitude(3), 15, -32];
initCovarianceUKF = 0.001*eye(6);

%% Particle filter
% num_particles = 100;
initEstimateParticleFilter = [0, 0, initGroundSpeedEst, initAttitude(3), 15, -30];


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
