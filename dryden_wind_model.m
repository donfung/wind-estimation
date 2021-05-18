

T=2000;
dt=0.01;
mu_ug=0;
Q=1;
W = repmat(mu_ug,T,1) + randn(T,1)*(Q);

W_20=25;%knots wind speed at 20 feet
sigma_w=0.1*W_20; %RMS turbulence intensity


u_g(1,1)=0;
for k=1:T
V=10+sin(k);
h=100+cos(k);    
sigma_v=sigma_w/(0.177+0.000823*h)^0.4;% RMS turbulence intensity

sigma_u=sigma_v; %RMS turbulence intensity
L_u=h/(0.177+0.000823*h)^1.2;
u_g(k+1,1)=(1-V*dt/L_u)*u_g(k,1)+sqrt(2*V*dt/L_u)*sigma_u/W(k,1);
end
plot(u_g(1:T));