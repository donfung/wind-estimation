clear all;
air_speed=10;
altitude=300;
mu=0;
phi=10;
theta=10;
psi=10;
V_pred=[1;1;1];
%v_wind_speed=[10;1;0;1;1;1];
airspeed=[100;1;1];
v_wind_speed=zeros(1,6)';
for k=1:1000
   [v_wind_dryden,A]= dryden_wind_model(airspeed,v_wind_speed(:,k),altitude,phi,theta,psi);
    v_wind_speed(:,k+1)=v_wind_dryden;
end
