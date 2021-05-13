mu_w_k=exp(-Ts/tau)*mu_k_prev+w_k_gm;
% tau- correlation time of the GM process, w_k_gm is the driven noise with
% variance given
sigma_gm=sigma_b^2(1-2^(2Ts/tau));
