%%% This file contains mock data to demonstrate the following filters
% - Kalman filter (linear)
% - Extended Kalman filter
% - Iterated Extended Kalman Filter
% - Unscented Kalman Filter
% - Particle Filter

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
    % can be a nonlinear g function
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

%% Kalman filter (linear)
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

%% iEKF
function [mu_k_k,Sig_k_k] = iEKF(mu_km_km, Sig_km_km, u_km, y_k, dynamics, measurement, A_k, C_k, Q, R, verbose)
    % predict step
    mu_k_km = dynamics(mu_km_km, u_km);
    Sig_k_km = A_k*Sig_km_km*A_k' + Q;
    
    % update step
    mu_k_old = mu_k_km; i = 0;
    converged = false; 
    while ~converged
        ybar_k = y_k - measurement(mu_k_old, u_km);
        C_k = C(mu_k_old);
        K_k = Sig_k_km * C_k' / (C_k*Sig_k_km*C_k'+R);
        mu_k_new = mu_k_km + K_k*ybar_k + K_k*C_k*(mu_k_old - mu_k_km);
        if (mu_k_new == mu_k_old)
            converged = true;
            if verbose
                fprintf('iEKF: Converged at iteration %i \n',i);
            end
        end
        if i>100
            converged = true;
            if verbose
                fprintf('iEKF: Timing out after 100 iterations\n');
            end
        end
        i = i+1;
        mu_k_old = mu_k_new;        
    end
    mu_k_k = mu_k_new;
    Sig_k_k = Sig_k_km - K_k*C_k*Sig_k_km;
end

%% Unscented Kalman Filter
function [mu_k_k,Sig_k_k] = UKF(mu_km_km, Sig_km_km, u_km, y_k, dynamics, measurement, Q, R, verbose)
    global xdim; global ydim;
    % predict step
    % mu_km_km is of size (n,1), prior_points is of size (n,2n+1), weights is of size (2n+1,1)
    [prior_points, w0, wi] = UT(mu_km_km, Sig_km_km);
    pred_points = prior_points;
    for i = 1:2*xdim+1
        pred_points(:,i) = dynamics(prior_points(:,i), u_km);
    end
    [mu_k_km, Sig_k_km] = UTinv(pred_points, w0, wi, Q);
    
    % update step
    [update_points, w0, wi] = UT(mu_k_km, Sig_k_km); % update_points and pred_points same?
    y_true = zeros(ydim,2*xdim+1);
    y_est = zeros(ydim,1);
    for i = 1:size(update_points,2)
        y_true(:,i) = measurement(update_points(:,i),u_km);
    end
    y_est = w0*y_true(:,1) + wi*sum(y_true,2);
    Cov_Y = zeros(ydim,ydim);
    Cov_XY = zeros(xdim,ydim);
    for i = 1:2*xdim+1
        if i == 1
            wt_i = w0;
        else
            wt_i = wi;
        end
        Cov_Y = Cov_Y + wt_i*(y_true(:,i)-y_est)*(y_true(:,i)-y_est)';
        Cov_XY = Cov_XY + wt_i*(update_points(:,i)-mu_k_km)*(y_true(:,i)-y_est)';
    end
    Cov_Y = Cov_Y + R;
    mu_k_k = mu_k_km + Cov_XY*inv(Cov_Y)*(y_k - y_est);
    Sig_k_k = Sig_k_km - Cov_XY*inv(Cov_Y)*Cov_XY'; 
end

%% Helper functions for UKF
% mu_km_km is of size (xdim,1), prior_points is of size (xdim,2n+1)
function [prior_points, w0, wi] = UT(mu, Sig)
    lambda = 2;
    global xdim;
    prior_points = zeros(xdim, 2*xdim+1);
    % weights
    w0 = lambda/(lambda+xdim);
    wi = 1/(2*(lambda+xdim));
    % 0 point
    prior_points(:,1) = mu;
    matrix_sqr_root = sqrtm((lambda+xdim)*Sig);
    for i = 1:xdim
        % ith point
        prior_points(:,1+i) = mu + matrix_sqr_root(:,i);
        % i + nth point
        prior_points(:,1+i+xdim) = mu - matrix_sqr_root(:,i);
    end
end

% points is of size (n,2n+1)
function [mu, Sig] = UTinv(points, w0, wi, Q)
    global xdim;
    n = xdim;
    mu = zeros(n,1);
    Sig = zeros(n,n);
    mu = w0*points(:,1) + wi*sum(points,2);
    for i = 1:2*n+1
        if i == 1
            Sig = Sig + w0*(points(:,i) - mu)*(points(:,i) - mu)';
        else
            Sig = Sig + wi*(points(:,i) - mu)*(points(:,i) - mu)';
        end
    end
    Sig = Sig + Q;
end

%% Particle filter
function [mu_k_k, Sig_k_k] = particle_filter(mu_km_km, Sig_km_km, u_km, y_k, dynamics_with_noise, measurement, Q, R, verbose)
    num_particles = 1000;
    % sample particles
    prior_points = make_particles(mu_km_km,Sig_km_km,num_particles);    
    % predict step
    pred_points = prior_points;
    unnorm_weights = zeros(num_particles);
    inv_R = inv(R); % precalculate just once
    for i = 1:num_particles
        % predict points
        pred_points(:,i) = dynamics_with_noise(prior_points(:,i), u_km, Q);
        % update weights
        unnorm_weights(i) = gaussian_pdf(y_k, measurement(pred_points(:,i), u_km), inv_R);
    end
    norm_weights = unnorm_weights/sum(unnorm_weights);
    % resample particles 
    post_points = resample_particles(norm_weights, pred_points);
    [mu_k_k, Sig_k_k] = get_mu_sig_from_particles(post_points); 
end

%% Helper functions for particle filter
function particles = make_particles(mu,Sig,num_particles)
    % create 1000 particles
    global xdim;
%     particles = zeros(xdim, num_particles); 
    particles = repmat(mu,1,num_particles) + chol(Sig)*randn(xdim,num_particles);
end

function [mu,Sig] = get_mu_sig_from_particles(particles)
    mu = mean(particles,2);
    Sig = cov(particles');
end

function new_particles = resample_particles(weights, particles)
    % points ~(xdim, num_particles)
    % weights ~(num_particles,1)
    global xdim;
    num_particles = size(particles,2);
    % Get new indices
    inds = randsample(1:num_particles, num_particles, true, weights);
    new_particles = particles(:,inds);
end

function likelihood = gaussian_pdf(x, mu, inv_Sig)
    resid = x - mu;
    likelihood = exp(-0.5*resid'*inv_Sig*resid);     
end