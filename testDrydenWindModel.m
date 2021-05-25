clear all;
air_speed=10;
altitude=300;
mu=0;
alpha=10;
beta=10;
ut=1;
vt=1;
wt=1;
for k=1:1000
    
   [v,A]= dryden_wind_model(air_speed,altitude,mu,mu,mu,alpha,beta,ut,vt,wt);
end
