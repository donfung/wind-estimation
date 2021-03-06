function [v_wind_ned,A,Q_process] = wind_model(X)
%wind model calls the dryden model and 
v_aircraft_NED(1,1)=X(4,1); 
v_aircraft_NED(2,1)=X(5,1);
v_aircraft_NED(3,1)=X(6,1);
phi=X(7,1);
theta=X(8,1);
psi=X(9,1);
v_wind=X(13:18,1);
altitude=100;
[v_wind_ned,A,Q_process] = dryden_wind_model(v_aircraft_NED,v_wind,altitude,phi,theta,psi);
%outputArg2 = inputArg2;
end

