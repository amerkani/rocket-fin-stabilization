close all
warning('off','all')
%% Continuous Time

clear

Ix = .1;                 % rotational moment of inertia
C_l_prime = .01;        % slope of cl vs alpha curve
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
C = [1 0; 0 1];
D = 0;

q1 = 200; 
r1 = 1;

Q = C'*q1*C;
R = r1;


K = lqr(A, B, Q, R);

sys1 = ss(A-B*K, B, C, D);
step(sys1)

x0 = [0 20]';
[T,X] = ode45(@stabilize, 0:.001:4, x0);

plot(T, X(:,2))

%% Discrete Time Updates

dT = .004;           % Sample rate (control loop time)
totalT = 8; %secs
stepsPerdt = 10;
[Kd, Sd, ed] = lqrd(A,B,Q,R, dT)
rate = 135; % deg/s (servo)


tauVecLength = totalT / dT * (stepsPerdt + 1);
time = linspace(0,totalT,tauVecLength);

tauConst = 5*ones(tauVecLength,1) ;
tauPeriodic = 2.5 * sin(time) + 5;
seedPseudo = randi([1000,2000],1,60);
tauPseudoRand = [];
for i = 1:1:length(seedPseudo)
    if mod(i,2) == 0
        tauPseudoRand = [tauPseudoRand, zeros(1,seedPseudo(i))];
    else
        tauPseudoRand = [tauPseudoRand, ones(1,seedPseudo(i))*5];
    end
end

tauChoice = tauPseudoRand(1:tauVecLength);
    
tau = reshape(tauChoice, [(stepsPerdt +1), totalT/dT]);

u=0;
uf=0;
xp = [0; 0]; xc = [0;0]; r = 0;
ti = 0; tf = ti + dT; Traj = [];
x_true = xp;
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
  
    pertPath = u_path + tau(:,k);
    [Yout, Tout, Xout]=lsim(ss(A,B,C,D),pertPath,timespan,x_true);
%     [Tout, Xout] = ode45(@(t,x) stabilize_pert(t,x, u0, uf, timespan, dT, rate), timespan, xp);
    xp = Xout(end,:)' + [angle_noise*randn; rate_noise*randn];
    Traj = [Traj; Tout(:), Xout, xp'.*ones(size(Xout)), u_path];
    
    uf = u_path(end);
    ti = tf;
    tf = ti+dT;
    x_true = Xout(end,:)';

end
%ang vel
figure
plot((Traj(:,1)),Traj(:,5),'g', (Traj(:,1)),Traj(:,3),'b',flip(Traj(:,1)),r*ones(size(Traj(:,1))), 'r--','linewidth',2);
set(gca,'fontsize', 16);
xlabel('Time (s)');
ylabel('rad/sec')
legend('Measured Roll Rate', 'True Roll Rate', 'Reference');

set(gcf, 'Position', get(0, 'Screensize').*0.4 + [400 300 200 100]); 
saveas(gcf, '/Users/shantamerkanian/Desktop/School/W23/AE 405/Final Report/simulated_tauPseudoRand.png')

figure
plot((Traj(:,1)),Traj(:,2),'b', 'linewidth',3);
set(gca,'fontsize', 16);
title('Pseudo Random: Angle')
xlabel('Time (s)');
ylabel('rad')
legend('Angle');

figure
plot((Traj(:,1)), Traj(:,6), 'b', time, tauChoice, 'r--','linewidth',3);
set(gca,'fontsize', 16);
title('Pseudo Random: Controls')
xlabel('Time (s)');
ylabel('deg')
legend('u');
