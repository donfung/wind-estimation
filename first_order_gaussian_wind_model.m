mu_w(:,1)=[0; 0; 0];
sigma=[1e-6 0 0;0 1e-6 0;0 0 1e-6];
R=chol(sigma);
z = repmat(mu_w(:,1)',100,1) + randn(100,3)*R;
for i=1:100
   mu_w(:,i+1)=mu_w(:,i)+z(i,:)';
end