
v_aircraft_ned=[100;0;0];
psi=0;
theta=0;
phi=0;
v_wind_ned=[0;0;0;0;0;0];
altitude=1000;

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
u_t_ned_temp=(1-V_airspeed_body_norm_knots*dt/L_u)*u_t_ned+sqrt(2*V_airspeed_body_norm_knots*dt/L_u)*sigma_u*noise;
u_t_ned=u_t_ned_temp*0.5144; % ug in m/s

v_t_ned_temp=(1-V_airspeed_body_norm_knots*dt/L_u)*v_t_ned+sqrt(2*V_airspeed_body_norm_knots*dt/L_u)*sigma_v*noise;
v_t_ned=v_t_ned*0.5144; % ug in m/s

w_t_ned_temp=(1-V_airspeed_body_norm_knots*dt/L_u)*w_t_ned+sqrt(2*V_airspeed_body_norm_knots*dt/L_u)*sigma_w*noise;
w_t_ned=w_t_ned*0.5144; % ug in m/s

%% assgining the new turbulent speeds to the return vector
v_w_ned(4,1)= u_t_ned;
v_w_ned(5,1)= v_t_ned;
v_w_ned(6,1)= w_t_ned;
