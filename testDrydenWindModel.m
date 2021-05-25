clear all;
air_speed=10;
altitude=300;
mu=0;
alpha=10;
beta=10;
V_pred=[1;1;1];
mu_vt_pred=[1;1;1];

for k=1:1000 
   [v_turbulent_velocities,A]= dryden_wind_model(air_speed,altitude,mu_vt_pred,alpha,beta);
end
