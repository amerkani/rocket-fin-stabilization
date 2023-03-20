close all
warning('off','all')
%% Trial 2 - basic

clear

Ix = .1;                 % rotational moment of inertia
Iy = 1e6;                 % rotational moment of inertia
Iz = 1e6;                 % rotational moment of inertia
m = 10;                 % mass of the rocket in kg
C_l_prime = .01;        % slope of cl vs alpha curve
C_d_fit = [1.294e-4 1.0734e-5 6.972e-3];% quadratic fit coefficients for C_d vs alpha
rho = 1.17;             % density in kg/m^3
S = .007;               % wing area in m^2
area = .0001;              % wing cross-sectional area
d_l = 0.15;             % distance from roll axis to aerodynamic center

angle_noise = 0.003;
rate_noise = .06;

v = 70;                 % Air Speed (m/s)

A = [
    0 1;
    0 0;
];
B = [
    0;
    1.5*rho*v^2*C_l_prime*S*d_l*(1/Ix)
];
C = [1 1];
D = 0;

q1 = 20; % change this for parameters
r1 = 1;

Q = C'*q1*C
R = r1;

K = lqr(A, B, Q, R);

sys1 = ss(A-B*K, B, C, D);
% step(sys1)

x0 = [0 20]';
% [T,X] = ode45(@stabilize, 0:.001:4, x0);

% plot(T, X(:,2))

% ideal continuous 

%% Discrete Time Updates

dT = .004;           % Sample rate (control loop time)
totalT = 20; %secs
stepsPerdt = 10;
[Kd, Sd, ed] = lqrd(A,B,Q,R, dT);
rate = 135; % deg/s (servo)

% Kalman
plant = ss(A,B,C,D,dT);
Qk = 0;
Rk = [
    angle_noise*10 0;
    0 rate_noise;
];
Rk = rate_noise;
[kalmf,L,~,Mx,Z] = kalman(plant,Qk,Rk);


%tau = ones(size(timespan))*4;
tauVecLength = totalT / dT * (stepsPerdt + 1);
time = linspace(0,totalT,tauVecLength);

tauConst = 10*ones(tauVecLength,1) ;
tauPeriodic = 10 * sin(time) + 1;
seedPseudo = randi([2000,4000],1,30);
tauPseudoRand = [];
for i = 1:1:length(seedPseudo)
    if mod(i,2) == 0
        tauPseudoRand = [tauPseudoRand, zeros(1,seedPseudo(i))];
    else
        tauPseudoRand = [tauPseudoRand, ones(1,seedPseudo(i))*10];
    end
end
    
tau = reshape(tauConst(1:tauVecLength), [(stepsPerdt +1), totalT/dT]);

u=0;
uf=0;
xp = [0; 0]; xc = [0;0]; r = 0;
ti = 0; tf = ti + dT; Traj = [];
% P = 1/s^2/(s+5); [A,B,C,D]=ssdata(P);
for k=1:1:totalT/dT

    u0 = uf;
    u = -Kd*xp;
    if abs(u) > 15
        u = (abs(u)/u)*15;
    end
    %u = 0;
    if u == u0
        dir = 0;
    else
        dir = abs(u-u0)/(u-u0);
    end
    
%u 
    timespan = [ti:dT/stepsPerdt:tf]';
    
    % 
    u_path = u0 + (timespan-ti)*dir*rate;
    for i=1:size(u_path)
        if dir < 0
            if u_path(i) < u
                u_path(i) = u;
            end
        elseif dir > 0
            if u_path(i) > u
                u_path(i) = u;
            end
        end
    end
  
    %u_path
    %tau = 0;
    % change tau -move periodically 
    pertPath = u_path + tau(:,k);
    %u_path = u_path +1;
    [Yout, Tout, Xout]=lsim(ss(A,B,C,D),pertPath,timespan,xp);
%     [Tout, Xout] = ode45(@(t,x) stabilize_pert(t,x, u0, uf, timespan, dT, rate), timespan, xp);
    xp = Xout(end,:)' + [angle_noise*randn; rate_noise*randn];
    Traj = [Traj; Tout(:), Xout, u_path];
    
    uf = u_path(end);
    ti = tf;
    tf = ti+dT;

end
%ang vel
figure
plot((Traj(:,1)),Traj(:,3),'b',flip(Traj(:,1)),r*ones(size(Traj(:,1))), 'r--','linewidth',3);
set(gca,'fontsize', 16);
title('Pseudo Random: Angular Velocity')
xlabel('Time (s)');
ylabel('rad/sec')
legend('y', 'r');
%no tau = no control 
%contr
figure
plot((Traj(:,1)), Traj(:,4), 'b', 'linewidth',3);
set(gca,'fontsize', 16);
title('Pseudo Random: Controls')
xlabel('Time (s)');
ylabel('deg')
legend('u');

%% Functions
