
function[ug]=dryden_wind_model(air_speed,altitude,ug_prev)

V=air_speed*1.943; % airspeed in knots
h=altitude*3.28; %height in ft

dt=0.1;
mu_ug=0;
Q=2;
noise=randn(1,1)*Q;
W_20=25;%knots wind speed at 20 feet

sigma_w=0.1*W_20; %RMS turbulence intensity  
sigma_v=sigma_w/(0.177+0.000823*h)^0.4;% RMS turbulence intensity
sigma_u=sigma_v; %RMS turbulence intensity

L_u=h/(0.177+0.000823*h)^1.2; %turbulence length scale
ug=(1-V*dt/L_u)*ug_prev+sqrt(2*V*dt/L_u)*sigma_u*noise;
ug=ug*0.5144; % ug in m/s

end