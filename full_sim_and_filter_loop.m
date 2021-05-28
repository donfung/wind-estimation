%% Full sim and filtering
clc
clear all
close all

% All units are SI units!!
% u, v, w: velocity of aircraft relative to Earth in body frame
% u_r: velocity of aircraft relative to surrounding airmass
% u_r_m: measurement of u_r
% airplane vector = [x; y; z; u; v; w; phi; theta; psi; p; q; r] 
% wind vector = [windN_s; windE_s; windD_s; windN_t; windE_t; windD_t]
% state vector = airplane vector + wind vector

% measurement vector = [IMU, GPS, airspeed, altimeter]
% Initial measurement vector = [u; v; w; phi; theta; psi; u_r_m; h]

dt = 0.01; T = 20; t = 0:dt:T;
global state_dim; global action_dim; global y_dim; global wind_state_dim;
state_dim = 18; 
wind_state_dim = 6;
action_dim = 4; % [da, de, dT, dr]
y_dim = 4; 
global N;
N = length(t);
state_init = zeros(state_dim,1);
state = zeros(state_dim, N+1);
state(:,1) = state_init;
y = zeros(y_dim, N);
action = zeros(action_dim, N);

R = 0.1*eye(y_dim); % TODO: what is reasonable?

mu_0 = zeros(state_dim,1);
Sig_0 = 0.1*eye(state_dim);

mu_values = zeros(state_dim, N+1);
mu_values(:,1) = mu_0;
Sig_values = zeros(state_dim, state_dim, N+1);
Sig_values(:,:,1) = Sig_0;

% Simulate full states
for i = 2:N
    action(:,i) = PID('cruise');
    cur_time = dt*(i-1);
    Q = Q_matrix(cur_time);
    state(:,i) = dynamics_with_noise(state(:,i-1),action,Q);
    y(i) = measurement_with_noise(state(:,i), R);
end

% Filtering 
for i = 2:N
    mu_prev = mu_values(:,i-1);
    Sig_prev = Sig_values(:,:,i-1);
    cur_time = dt*(i-1);
    Q = Q_matrix(cur_time);
    [mu_new, Sig_new] = extended_Kalman_filter(mu_prev, Sig_prev, action(:,i-1), y(i), dynamics, measurement, A, C, Q, R);
    mu_values(:,i) = mu_new;
    Sig_values(:,:,i) = Sig_new;
end

function Q = Q_matrix(time)
    global state_dim; global wind_state_dim;
    Q = zeros(state_dim, state_dim);
    Q(end-wind_state_dim+1:end, end-wind_state_dim+1:end) = wind_Q_matrix(time);
end

function wind_Q = wind_Q_matrix(time)
    % TODO (Vishnu) Get Q for wind vector
    global wind_state_dim;
    wind_Q = time*1/100*eye(wind_state_dim, wind_state_dim);
end

% maneuver_type can be cruise, climb, descend, turn
function action = PID(maneuver_type)
    % TODO (Don)
    error('Not implemented.');
end

function state_new = dynamics_with_noise(state, action, Q)
    % TODO (somrita
    process_noise = (randn(1, state_dim)*sqrtm(Q))';
    state_new = dynamics(state, action) + process_noise;
end

function state_new = dynamics(state, u)
    % lateral and longitudinal of Navion combined into 1 matrix 
    % TODO (somrita)
    % Also calls wind_dynamics for 2nd part of state
    % non linear function f
    A = [0.8 0 0.001; 0.0 0.6 0.0; 0.0 0.0 0.9]; B = [0.5; 0.2; 0.7];
    state_new = A*state + B*u;
end

% Input state: full 18 dim
% Output wind_state_new: 6 dim 
    % wind vector = [windN_s; windE_s; windD_s; windN_t; windE_t; windD_t])
function wind_state_new = wind_dynamics(state, u)
    % Will call Dryden model, should eventually move to another file
    % Process state to inputs required by dryden_wind_model
    % Get vt_nonlinear_model and propagate (dt) to return state_new
    % TODO (Vishnu)
end

function y = measurement_with_noise(state, R)
    % sensor noise - equation 34 of Wenz 2016
    % TODO (Don)
    meas_noise = (randn(1,y_dim)*sqrtm(R))';
    y = measurement(state) + meas_noise;
end

function y = measurement(state)
    % sensor models - equation 35 of Wenz 2016
    % TODO (Don)
    % can be a nonlinear g function
    C = [0.4 0.9 0.1; 0.5 0.2 0.5];
    y = C*state;
end

