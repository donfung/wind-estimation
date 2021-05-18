%%% This file contains mock data to demonstrate the following filters
% - Kalman filter (linear)
% - Extended Kalman filter

% Mock data
dt = 0.1; N = 10*1/dt;
state_dim = 3; obs_dim = 2;
state_0 = [0; 0; 0];
Q = 0.1*eye(state_dim);
R = 0.1*eye(obs_dim);
A = [0.8 0 0.001; 0.0 0.6 0.0; 0.0 0.0 0.9];
B = [0.5; 0.2; 0.7];
C = [0.4 0.9 0.1; 0.5 0.2 0.5];
mu_0 = [0;0;0];
Sig_0 = diag([1;1;1]);
state_values = zeros(state_dim, N+1);
state_values(:,1) = state_0;
y_values = zeros(obs_dim, N);
mu_values = zeros(state_dim, N+1);
mu_values(:,1) = mu_0;
Sig_values = zeros(state_dim, state_dim, N+1);
Sig_values(:,:,1) = Sig_0;

for k = 1:N
    u = control_input(k);
    state_values(:,k+1) = dynamics_with_noise(state_values(:,k), u, Q);
    y_values(:,k) = measurement_with_noise(state_values(:,k+1), R);
    mu_prev = mu_values(:,k);
    Sig_prev = Sig_values(:,:,k);
    
    [mu_new, Sig_new] = kalman_filter(mu_prev, Sig_prev, u, y_values(:,k), A, B, C, Q, R);
    mu_values(:,k+1) = mu_new;
    Sig_values(:,:,k+1) = Sig_new;
end
time_vec = 0:dt:N*dt;
figure
for i = 1:state_dim
    subplot(state_dim,1,i)
    h = plot_state(time_vec, state_values, mu_values, i);
end

function u = control_input(t)
    u = 1;
end

function state_new = dynamics_with_noise(state, u, Q)
    process_noise = (randn(1,3)*sqrtm(Q))';
    state_new = dynamics(state, u) + process_noise;
end

function state_new = dynamics(state, u)
    A = [0.8 0 0.001; 0.0 0.6 0.0; 0.0 0.0 0.9]; B = [0.5; 0.2; 0.7];
    state_new = A*state + B*u;
end

function y = measurement_with_noise(state, R)
    meas_noise = (randn(1,2)*sqrtm(R))';
    y = measurement(state) + meas_noise;
end

function y = measurement(state)
    C = [0.4 0.9 0.1; 0.5 0.2 0.5];
    y = C*state;
end
  
function h = plot_state(time_vec, state_values, mu_values, ind)
    h = plot(time_vec, state_values(ind,:), 'DisplayName', 'True');
    hold on;
    plot(time_vec, mu_values(ind,:), 'DisplayName', 'KF estimate');
    xlabel('Time [s]'); 
    if ind == 1
        ylabel('State[1]');
    elseif ind == 2
        ylabel('State[2]');
    elseif ind == 3
        ylabel('State[3]');
    end
    legend show; grid on;
end


% Inputs: mu_k, Sig_k, u_k, y_kp, A, B, Q, C, R
% Outputs: mu_kp, Sig_kp
function [mu_kp, Sig_kp] = kalman_filter(mu_k, Sig_k, u_k, y_kp, A, B, C, Q, R)
    % Prediction step
    mu_pred = A*mu_k + B*u_k;
    Sig_pred = A*Sig_k*A' + Q;

    % Update step
    y_bar = y_kp - C*mu_pred;
    K = Sig_pred*C'/(C*Sig_pred*C' + R); % Kalman gain
    mu_kp = mu_pred + K*y_bar;
    Sig_kp = Sig_pred - K*C*Sig_pred;
end

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
