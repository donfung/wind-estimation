%% Full sim and filtering
clc
clear all
close all

% All units are SI units!!
% u, v, w: velocity of aircraft relative to Earth in body frame
% u_r: velocity of aircraft relative to surrounding airmass
% u_r_m: measurement of u_r
% airplane vector = [x; y; z; u; v; w; phi; theta; psi; p; q; r; h] 
% wind vector = [windN_s; windE_s; windD_s; windN_t; windE_t; windD_t]
% state vector = airplane vector + wind vector

% measurement vector = [IMU, GPS, airspeed, altimeter]
% Initial measurement vector = [u; v; w; phi; theta; psi; u_r_m; h]

global u0;
u0 = 53.6448; % m/s which is 176 ft/s
global dt;
dt = 0.01; T = 20; t = 0:dt:T;
global state_dim; global action_dim; global y_dim; global wind_state_dim;
state_dim = 19; 
wind_state_dim = 6;
action_dim = 4; % [da, de, dT, dr]
y_dim = 4; 
global N;
N = length(t);
state_init = zeros(state_dim,1);
state = zeros(state_dim, N+1);
state(:,1) = state_init;
y = zeros(y_dim, N+1);
action = zeros(action_dim, N);

R = 0.1*eye(y_dim); % TODO: what is reasonable?

mu_0 = zeros(state_dim,1);
Sig_0 = 0.1*eye(state_dim);

mu_values = zeros(state_dim, N+1);
mu_values(:,1) = mu_0;
Sig_values = zeros(state_dim, state_dim, N+1);
Sig_values(:,:,1) = Sig_0;

% Simulate full states
for i = 2:N+1
    action(:,i) = PID('cruise');
    cur_time = dt*(i-1);
    Q = Q_matrix(cur_time);
    state(:,i) = dynamics_with_noise(state(:,i-1),action,Q);
    y(:,i) = measurement_with_noise(state(:,i), R);
end

figure 
plot3(state(1,:), state(2,:), state(3,:))
xlabel('x');
ylabel('y');
zlabel('z');
title('Aircraft position');
grid on;

figure 
plot(state(13,:))
ylabel('h');
xlabel('time');
title('Aircraft height');
grid on;

[A,B] = full_A_B();
C = zeros(y_dim, state_dim); % TODO (Don)
% Filtering 
for i = 2:N+1
    mu_prev = mu_values(:,i-1);
    Sig_prev = Sig_values(:,:,i-1);
    cur_time = dt*(i-1);
    Q = Q_matrix(cur_time);
    [mu_new, Sig_new] = extended_Kalman_filter(mu_prev, Sig_prev, action(:,i-1), y(i), @dynamics, @measurement, A, C, Q, R);
    mu_values(:,i) = mu_new;
    Sig_values(:,:,i) = Sig_new;
end

% figure 
% plot3(state(1,:), state(2,:), state(3,:))
% xlabel('x');
% ylabel('y');
% zlabel('z');
% hold on
% plot3(mu_values(1,:), mu_values(2,:), mu_values(3,:), 'DisplayName','estimate');
% title('Aircraft position');
% legend show;
% grid on;

% figure 
% plot(state(13,:))
% hold on
% plot(mu_values(13,:))
% ylabel('h');
% xlabel('time');
% title('Aircraft height');
% grid on;

function Q = Q_matrix(time)
    global state_dim; global wind_state_dim;
    Q = 0.001*eye(state_dim, state_dim);
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
%     error('Not implemented.');
    action = zeros(4,1);
end

function state_new = dynamics_with_noise(state, action, Q)
    global state_dim;
    process_noise = (randn(1, state_dim)*sqrtm(Q))';
    state_new = dynamics(state, action) + process_noise;
end

function state_new = dynamics(state, action)
    global dt; global u0;
    global state_dim; global wind_state_dim; global action_dim;
    % ALL STATES AND CONTROLS
    % airplane vector = [x; y; z; u; v; w; phi; theta; psi; p; q; r; h] 
    % wind vector = [windN_s; windE_s; windD_s; windN_t; windE_t; windD_t]
    % state vector = airplane vector + wind vector
    % action vector = [da, de, dT, dr]
    
    % Longitudinal aircraft dynamics
    % x = [ubar, alpha, q, theta]^T   u = [de, dT]^T % xdot = Ax + Bu
    [long_A,long_B] = longitudinal_Navion_A_B();
    long_state = [state(4)/u0; state(6)/u0; state(11); state(8)];
    long_u = [action(2); action(3)];
    long_state_new = (long_A*long_state + long_B*long_u)*dt + long_state;
    % Lateral aircraft dynamics
    % x = [beta, p, r, phi, psi]^T   u = [da, dr]^T 
    [lat_A,lat_B] = lateral_Navion_A_B();
    lat_state = [state(5)/u0; state(10); state(12); state(7); state(9)];
    lat_u = [action(1); action(4)];
    lat_state_new = (lat_A*lat_state + lat_B*lat_u)*dt + lat_state;
    % Position
    old_xyz = state(1:3); old_uvw = state(4:6);
    new_xyz = old_xyz + dt*old_uvw;
    % Height 
    % hdot = -u0*alpha + u0*theta. Using old long_state on purpose.
    new_h = state(13) + dt*(-u0*long_state(2) + u0*long_state(4));
    % Wind dynamics
    wind_state_new = wind_dynamics(state, action);
    % Combining
    % state_new = [x; y; z; u; v; w; phi; theta; psi; p; q; r; h; windN_s; windE_s; windD_s; windN_t; windE_t; windD_t]
    state_new = [new_xyz; % x, y, z
        u0*long_state_new(1); % u
        u0*lat_state_new(1); % v
        u0*long_state_new(2); % w
        lat_state_new(4); % phi
        long_state_new(4);% theta
        lat_state_new(5); % psi
        lat_state_new(2); % p
        long_state_new(3); % q
        lat_state_new(3); % r
        new_h; % h
        wind_state_new];
end

function [A,B] = full_A_B() 
    global dt; global u0; global state_dim; global action_dim;
    A = zeros(state_dim, state_dim);
    B = zeros(state_dim, action_dim);
    % These are Abar from xdot = Abar*x + Bbar*u
    % Need x_new = Ax + Bu. x_new = (I + dt*Abar)*x + (dt*Bbar)*u
    [long_Abar,long_Bbar] = longitudinal_Navion_A_B();
    long_A = eye(4,4) + dt*long_Abar;
    long_B = dt*long_Bbar;
    [lat_Abar,lat_Bbar] = lateral_Navion_A_B();
    lat_A = eye(5,5) + dt*lat_Abar;
    lat_B = dt*lat_Bbar;
    % state_new = [x; y; z; u; v; w; phi; theta; psi; p; q; r; h; windN_s; windE_s; windD_s; windN_t; windE_t; windD_t]
    % Longitudinal aircraft dynamics
    % x = [ubar, alpha, q, theta]^T   u = [de, dT]^T % xdot = Ax + Bu
    A(4,4) = long_A(1,1); A(4,6) = long_A(1,2); A(4,11) = long_A(1,3); A(4,8) = long_A(1,4);
    A(6,4) = long_A(2,1); A(6,6) = long_A(2,2); A(6,11) = long_A(2,3); A(6,8) = long_A(2,4);
    A(11,4) = long_A(3,1); A(11,6) = long_A(3,2); A(11,11) = long_A(3,3); A(11,8) = long_A(3,4);
    A(8,4) = long_A(4,1); A(8,6) = long_A(4,2); A(8,11) = long_A(4,3); A(8,8) = long_A(4,4);
    % Lateral aircraft dynamics
    % x = [beta, p, r, phi, psi]^T   u = [da, dr]^T 
    A(5,5) = lat_A(1,1);    A(5,10) = lat_A(1,2);   A(5,12) = lat_A(1,3);   A(5,7) = lat_A(1,4);    A(5,9) = lat_A(1,5);
    A(10,5) = lat_A(2,1);   A(10,10) = lat_A(2,2);  A(10,12) = lat_A(2,3);  A(10,7) = lat_A(2,4);   A(10,9) = lat_A(2,5);
    A(12,5) = lat_A(3,1);   A(12,10) = lat_A(3,2);  A(12,12) = lat_A(3,3);  A(12,7) = lat_A(3,4);   A(12,9) = lat_A(3,5);
    A(7,5) = lat_A(4,1);    A(7,10) = lat_A(4,2);   A(7,12) = lat_A(4,3);   A(7,7) = lat_A(4,4);    A(7,9) = lat_A(4,5);
    A(9,5) = lat_A(5,1);    A(9,10) = lat_A(5,2);   A(9,12) = lat_A(5,3);   A(9,7) = lat_A(5,4);    A(9,9) = lat_A(5,5);
    % xyz
    A(1:3,1:3) = eye(3,3); A(1:3,4:6) = dt;
    % h hnew = h_old +  dt* hdot = h_old +  dt*(-u0*alpha + u0*theta) = h_old +  dt*(-w + u0*theta)
    A(13,13) = 1; A(13,6) = -dt; A(13,7) = dt*u0;
    % Wind
    A(14:19,14:19) = wind_A();
    % action vector = [da, de, dT, dr]
    % Finish B matrix
    % TODO(Somrita)
end

function A = wind_A()
    % TODO (Vishnu)
    % Should be 6x6
    A = eye(6,6);
end

% Input state: full 18 dim
% Output wind_state_new: 6 dim 
    % wind vector = [windN_s; windE_s; windD_s; windN_t; windE_t; windD_t])
function wind_state_new = wind_dynamics(state, action)
    % Will call Dryden model, should eventually move to another file
    % Process state to inputs required by dryden_wind_model
    % Get vt_nonlinear_model and propagate (dt) to return state_new
    % TODO (Vishnu)
    wind_state_new = zeros(6,1);
end

function y = measurement_with_noise(state, R)
    % sensor noise - equation 34 of Wenz 2016
    % TODO (Don)
    global y_dim;
    meas_noise = (randn(1,y_dim)*sqrtm(R))';
    y = measurement(state) + meas_noise;
end

function y = measurement(state)
    % sensor models - equation 35 of Wenz 2016
    % TODO (Don)
    % can be a nonlinear g function
    global state_dim; global y_dim;
    C = zeros(y_dim, state_dim);
    y = C*state;
end

%% Extended Kalman Filter
% Inputs: mu_k, Sig_k, u_k, y_kp, dynamics, measurement, A, C, Q, R
% Function signatures: dynamics(x,u) and measurement(x)
% A and C are Jacobians of dynamics and measurement respectively wrt state
% Outputs: mu_kp, Sig_kp
function [mu_kp, Sig_kp] = extended_Kalman_filter(mu_k, Sig_k, u_k, y_kp, dynamics, measurement, A, C, Q, R)
    % Prediction step
    mu_pred = dynamics(mu_k, u_k);
    Sig_pred = A*Sig_k*A' + Q;

    % Update step
    ybar = y_kp - measurement(mu_pred);
    K = Sig_pred*C'/(C*Sig_pred*C' + R); % Kalman gain
    mu_kp = mu_pred + K*ybar;
    Sig_kp = Sig_pred - K*C*Sig_pred;
end

