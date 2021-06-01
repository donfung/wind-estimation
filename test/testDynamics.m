clear all;    
clear hdePid;
close all;

dt = 0.01;
tFinal = 200;
time = 0 : dt : tFinal;
n = length(time);

[Alon, Blon] = longitudinal_Navion_A_B();
[Alat, Blat] = lateral_Navion_A_B();

xlon = zeros(4, n);  % xlon = [ubar, alpha, q, theta]
xlat = zeros(5, n);  % xlat = [beta, p, r, phi, psi]
h    = zeros(1, n);
hDot = zeros(1, n);

xlon(:, 1) = [1; 0; 0; 0];   % xlon = [ubar, alpha, q, theta]
xlat(:, 1) = [0; 0; 0; 0; 0];  % xlat = [beta, p, r, phi, psi]
h(1)       = -1000;

ulon = zeros(2, n);    % ulon = [de; dt]
ulat = zeros(2, n);    % ulat = [da; dr]

s = tf('s');

u0 = 176;  % Reference airspeed

C = [0, -u0, 0, u0;    % hdot
     0, 0, 0, 1;       % theta
     u0, 0, 0, 0];      % u

D = [0, 0];

G = tf(ss(Alon, Blon, C, zeros(size(C, 1), size(Blon, 2))));

hdotde  = G(1,1);
thetade = G(2,1);
ude     = G(3,1);
hdt     = G(1,2);
thetadt = G(2,2);
udt     = G(3,2);

hde = hdotde * 1/s;

hCmd = -1500;

for i = 2:n
    xlonDot = Alon * xlon(:, i-1) + Blon * ulon(:, i-1);
    xlon(:, i) = xlon(:, i-1) + xlonDot * dt;

%     xlatDot = Alat * xlat(:, i-1) + Blat * ulat(:, i-1);
    
%     xlon(:, i) = Alon * xlon(:, i-1) + Blon * ulon(:, i-1);
%     xlat(:, i) = Alat * xlat(:, i-1) + Blat * ulat(:, i-1);
    
%     xlat(:, i) = xlat(:, i-1) + xlatDot * dt;

    hDot(i) = -u0*xlon(2, i) + u0*xlon(4, i);  % positive up
    
    h(i) = h(i-1) - hDot(i)*dt;
    
    hErr = hCmd - h(i);
    
    deCmd = hdePid(hErr, dt);
    
    deMax = 45*pi/180;
    deMin = -45*pi/180;
    
    deCmdSat = sat(deCmd, deMax, deMin);
    
    ulon(:, i) = [deCmdSat, 0];
%     ulon(:, i) = [-5*pi/180, 10];
    
end

figure
subplot(4,1,1);
plot(time, 176*xlon(1, :)); title('u');
subplot(4,1,2);
plot(time, 180/pi*xlon(2, :)); title('alpha');
subplot(4,1,3);
plot(time, 180/pi*xlon(3, :)); title('q');
subplot(4,1,4);
plot(time, 180/pi*xlon(4, :)); title('theta');


figure;
plot(time, -h);
title('Altitude');

figure
plot(time, 180/pi*ulon(1, :))
title('Elevator Command (deg)');

function deCmd = hdePid(err, dt)
    % Gains from controlSystemDesigner(hde)
    K = -7.216e-5 * pi/180 *10^-3;
    Kp = K*14.9;
    Ki = K*1;
    Kd = K*42.9;
    Kd = 0;
    persistent integral isInitialized errPrev;
    
    if isempty(isInitialized)
        errPrev = 0;
        integral = 0;
        isInitialized = true;
    end
    
    integral = err + integral;
    derivative = (err - errPrev)/dt; 
    
    deCmd = Kp + Ki*integral + Kd*derivative;
    
    errPrev = err;
end

function saturated = sat(x, min, max)
    if x < min
        saturated = min;
    elseif x > max
        saturated = max;
    else
        saturated = x;
    end
end