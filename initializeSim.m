clear all;
close all;

%% Initialization script for the wind estimation simulation
% Below:
%       1. Initial conditions
%       2. Wind parameters
%       3. Simulation flags
%       4. EKF parameters
%       5. Sensor parameters

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
windNorthStatic = 15;
windEastStatic = -32;
windDownStatic = 0;

%% FLAGS
% 1 = enable. 0 = disable
sensorNoiseFlag     = 1;  % Sensor noises
turbulentWindFlag   = 1;  % Turn wind turbulence on/off 
staticWindFlag      = 0;  % Turn static wind on/off
gustWindFlag        = 0;  % Turn gust (step changes) on/off

%% EKF PARAMETERS
% mu = [pn, pe, vg, chi, wn, we]

Qekf = 1*eye(nEstStates);
Rekf = 1*eye(7);

initEstimateEkf = [0, 0, initGroundSpeedEst, initAttitude(3), 0, 0, 0, 0];
initCovarianceEkf = 0.1*eye(nEstStates);

%% SENSOR PARAMETERS
suite = 'high';  % String: 'low', 'mid', or 'high'
var = getSensorVariances(suite);




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
