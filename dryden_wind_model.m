
function[v_wind_ned,A]=dryden_wind_model(v_aircraft_ned,v_wind_ned,altitude,phi,theta,psi)


%% Non Linear Model 

%% Inputs: air speed NED,wind vector NED,altitude,phi,theta,psi

%% Outputs: updated_turbulent_speeds_vector and Jacobian

%% Conversions from NED to Body Frame
%V=air_speed*1.943; % airspeed in knots,magnitude
%v_n=ground_speed;
%v_w=wind_speed;

R_bn= [cos(psi)*cos(theta) cos(psi)*sin(phi)*sin(theta)-cos(phi)*sin(psi) sin(phi)*sin(psi)+cos(phi)*cos(psi)*sin(theta);
    cos(theta)*sin(psi) cos(theta)*cos(psi)+sin(phi)*sin(psi)*sin(theta) cos(phi)*sin(psi)*sin(theta)-cos(psi)*sin(phi)];

%v_aircraft_body=R_bn*v_aircraft_NED;

v_aircraft_airspeed_ned=v_aircraft_ned-v_wind_ned(1:3); % airspeed - NED frame ; Needed for equation 29

v_aircraft_airspeed_body=R_bn*(v_aircraft_ned)-R_bn*v_wind_ned(1:3);% airspeed body frame

V_airspeed_body_norm=norm(v_aircraft_airspeed_body);

V_airspeed_body_norm_knots=V_airspeed_body_norm*1.943;

%% wind speed dryden non linear model
h=altitude*3.28; %height in ft required by dryden model
u_r=10;% where do we get this from??
dt=0.1;
mu_ug=0;
Q=2;
noise=randn(1,1)*Q;
W_20=25;%knots wind speed at 20 feet

sigma_w=0.1*W_20; %RMS turbulence intensity  
sigma_v=sigma_w/(0.177+0.000823*h)^0.4;% RMS turbulence intensity
sigma_u=sigma_v; %RMS turbulence intensity

%%Turbulence length scales in u,v,w directions
L_w=h;
L_u=h/(0.177+0.000823*h)^1.2; %turbulence length scale
L_v=L_u;

%% turbulence wind components of the wind vector
u_t_ned = v_wind_ned(4,1);
v_t_ned = v_wind_ned(5,1);
w_t_ned=  v_wind_ned(6,1);

%% Non Linear Discrete Dryden model to propagate the turbulence velocity states
u_t_ned=(1-V_airspeed_body_norm_knots*dt/L_u)*u_t_ned+sqrt(2*V_airspeed_body_norm_knots*dt/L_u)*sigma_u*noise;
u_t_ned=u_t_ned*0.5144; % ug in m/s

v_t_ned=(1-V_airspeed_body_norm_knots*dt/L_u)*v_t_ned+sqrt(2*V_airspeed_body_norm_knots*dt/L_u)*sigma_v*noise;
v_t_ned=v_t_ned*0.5144; % ug in m/s

w_t_ned=(1-V_airspeed_body_norm_knots*dt/L_u)*w_t_ned+sqrt(2*V_airspeed_body_norm_knots*dt/L_u)*sigma_w*noise;
w_t_ned=w_t_ned*0.5144; % ug in m/s

%% assgining the new turbulent speeds to the return vector
v_wind_ned(4,1)= u_t_ned;
v_wind_ned(5,1)= v_t_ned;
v_wind_ned(6,1)= w_t_ned;

%% Jacobian
J=[1-dt*(V_airspeed_body_norm/L_u)  1-dt*(V_airspeed_body_norm/L_v) 1-dt*(V_airspeed_body_norm/L_w) ];

delF_delVt=diag(J);  % Equation 27 % partial derivatives with respect to high frequency turbulent component

delVa_delUs=(R_bn*[1 0 0]')'*v_aircraft_airspeed_body/V_airspeed_body_norm; % Equation 29

delVa_delVs=(R_bn*[0 1 0]')'*v_aircraft_airspeed_body/V_airspeed_body_norm;  % Equation 29

delVa_delWs=(R_bn*[0 0 1]')'*v_aircraft_airspeed_body/V_airspeed_body_norm; % Equation 29

delF_delVs=[ (dt/L_u)*(delVa_delUs)*u_t_ned (dt/L_u)*(delVa_delVs)*u_t_ned (dt/L_u)*(delVa_delWs)*u_t_ned;
    (dt/L_u)*(delVa_delUs)*v_t_ned (dt/L_u)*(delVa_delVs)*v_t_ned (dt/L_u)*(delVa_delWs)*v_t_ned;
    (dt/L_u)*(delVa_delUs)*w_t_ned (dt/L_u)*(delVa_delVs)*w_t_ned (dt/L_u)*(delVa_delWs)*w_t_ned]; %Equation 30 partial with respect to slow varying turbulence component

%% Jacobian output Equation 25 ; We don't have the last column because our state vector doesn't include CL_0,CL_alpha,Gamma
A= [eye(3)            zeros(3,3) ;
    delF_delVs         delF_delVt ;
    zeros(3,3)          zeros(3,3)];


end