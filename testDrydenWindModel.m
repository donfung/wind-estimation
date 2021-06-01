clear all;

altitude=300;

phi=10;
theta=10;
psi=10;
airspeed=[100;1;1];

v_wind_speed=[10;1;2;1;3;4];%
X=[1;2;3;airspeed;phi;theta;psi;1;2;3;v_wind_speed];
for k=1:1000
   %[v_wind_dryden,A]= dryden_wind_model(airspeed,v_wind_speed(:,k),altitude,phi,theta,psi);
   [v_wind_dryden,A,Q_process]= wind_model(X);
   v_wind_speed(:,k+1)=v_wind_dryden;
end
plot(v_wind_speed(4,:));
hold on;
plot(v_wind_speed(5,:));
hold on;
plot(v_wind_speed(6,:));
legend('u_w','v_w','w_w')
hold off;
