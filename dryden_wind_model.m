
function[ug,A]=dryden_wind_model(air_speed,altitude,ug_prev, alpha, beta,u_t_pred,v_t_pred,w_t_pred)

V=air_speed*1.943; % airspeed in knots
h=altitude*3.28; %height in ft
u_r=10;
dt=0.1;
mu_ug=0;
Q=2;
noise=randn(1,1)*Q;
W_20=25;%knots wind speed at 20 feet

sigma_w=0.1*W_20; %RMS turbulence intensity  
sigma_v=sigma_w/(0.177+0.000823*h)^0.4;% RMS turbulence intensity
sigma_u=sigma_v; %RMS turbulence intensity
L_w=h;
L_u=h/(0.177+0.000823*h)^1.2; %turbulence length scale
L_v=L_u;
ug=(1-V*dt/L_u)*ug_prev+sqrt(2*V*dt/L_u)*sigma_u*noise;
ug=ug*0.5144; % ug in m/s

%% Jacobian
v=[1-dt*(V/L_u)  1-dt*(V/L_v) 1-dt*(V/L_w) ];
delF_delVt=diag(v); % partial derivatives with respect to high frequency turbulent component

R_wb= [cos(alpha)*cos(beta) sin(beta) sin(alpha)*cos(beta);
    -cos(alpha)*sin(beta) cos(beta) -sin(alpha)*sin(beta);
    -sin(alpha) 0 cos(alpha)];
R_bw=inv(R_wb);%DCM wind frame to body frame

V_r=[u_r;u_r*tan(alpha);V*sin(beta)];

delVa_delUs=(R_bw*[1 0 0]')'*V_r/V;

delVa_delVs=(R_bw*[0 1 0]')'*V_r/V;

delVa_delWs=(R_bw*[0 0 1]')'*V_r/V;

delF_delVs=[ (dt/L_u)*(delVa_delUs)*u_t_pred (dt/L_u)*(delVa_delVs)*u_t_pred (dt/L_u)*(delVa_delWs)*u_t_pred;
    (dt/L_u)*(delVa_delUs)*v_t_pred (dt/L_u)*(delVa_delVs)*v_t_pred (dt/L_u)*(delVa_delWs)*v_t_pred;
    (dt/L_u)*(delVa_delUs)*w_t_pred (dt/L_u)*(delVa_delVs)*w_t_pred (dt/L_u)*(delVa_delWs)*w_t_pred]; % partial with respect to slow varying turbulence component


A=[eye(3) zeros(3,3) zeros(3,3);
    delF_delVs         delF_delVt       zeros(3,3);
    zeros(3,3) zeros(3,3) eye(3,3)];


end