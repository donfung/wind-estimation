% Inputs: mu_k, Sig_k, u_k, y_kp, A, B, Q, C, R
% Outputs: mu_kp, Sig_kp
function [mu_kp, Sig_kp] = kalman_filter(mu_k, Sig_k, u_k, y_kp, A, B, Q, C, R)
    % Prediction step
    mu_pred = A*mu_k + B*u_k;
    Sig_pred = A*Sig_k*A' + Q;

    % Update step
    K = Sig_pred*C'/(C*Sig_pred*C' + R); % Kalman gain
    mu_kp = mu_pred + K*(y_kp - C*mu_pred);
    Sig_kp = Sig_pred - K*C*Sig_pred;
end