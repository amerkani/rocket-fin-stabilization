%% Defining Constants
Ix = .1;                 % rotational moment of inertia
Iy = 1e6;                 % rotational moment of inertia
Iz = 1e6;                 % rotational moment of inertia
m = 10;                 % mass of the rocket in kg
C_l_prime = .01;        % slope of cl vs alpha curve
C_d_fit = [1.294e-4 1.0734e-5 6.972e-3];% quadratic fit coefficients for C_d vs alpha
rho = 1.17;             % density in kg/m^3
S = .01;               % wing area in m^2
area = .0001;              % wing cross-sectional area
d_l = 0.15;             % distance from roll axis to aerodynamic center

v = 70;                 % Air Speed (m/s)

syms p q r

x0 = [.001 .001 .001]';

A = [
    0 -x0(3)*(Iz - Iy)/Ix -x0(2)*(Iz - Iy)/Ix;
    -x0(3)*(Ix - Iz)/Iy 0 -x0(1)*(Ix-Iz)/Iy;
    -x0(2)*(Iy-Ix)/Iz -x0(1)*(Iy-Ix)/Iz 0;
];
A = zeros(3,3);
B = [
    1.5*rho*v^2*C_l_prime*S*d_l/Ix;
    0;
    0;
];
C = [1 1 1];
D = 0;
q1 = 5;
r1 = 1;

Q = C'*q1*C
R = r1;

K = lqr(A, B, Q, R)

% x_0 = [1 0 0]'

sys1 = ss(A-B*K, B, C, D);
step(sys1)

%% Trial 2 - basic

clear

Ix = .1;                 % rotational moment of inertia
Iy = 1e6;                 % rotational moment of inertia
Iz = 1e6;                 % rotational moment of inertia
m = 10;                 % mass of the rocket in kg
C_l_prime = .01;        % slope of cl vs alpha curve
C_d_fit = [1.294e-4 1.0734e-5 6.972e-3];% quadratic fit coefficients for C_d vs alpha
rho = 1.17;             % density in kg/m^3
S = .01;               % wing area in m^2
area = .0001;              % wing cross-sectional area
d_l = 0.15;             % distance from roll axis to aerodynamic center

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

q1 = 5; % change this for parameters
r1 = 1;

Q = C'*q1*C
R = r1;

K = lqr(A, B, Q, R)

sys1 = ss(A-B*K, B, C, D);
% step(sys1)

x0 = [0 20]';
[T,X] = ode45(@stabilize, 0:.001:4, x0);

plot(T, X(:,2))

%% Discrete Time Updates

dT = .004;           % Sample rate (control loop time)
[Kd, Sd, ed] = lqrd(A,B,Q,R, dT);
rate = 135; % deg/s (servo)

u=0;
uf=0;
xp = [0; 5]; xc = [0;0]; r = 0;
ti = 0; tf = ti + dT; Traj = [];
% P = 1/s^2/(s+5); [A,B,C,D]=ssdata(P);
for k=1:1:5/dT

    u0 = uf;
    u = -Kd*xp;
    if abs(u) > 15
        u = (abs(u)/u)*15;
    end
    dir = abs(u-u0)/(u-u0);

    timespan = [ti:dT/10:tf]';
    tau = ones(size(timespan))*4;
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
    tau = 0;
    u_path = u_path + tau;
    [Yout, Tout, Xout]=lsim(ss(A,B,C,D),u_path,timespan,xp);
%     [Tout, Xout] = ode45(@(t,x) stabilize_pert(t,x, u0, uf, timespan, dT, rate), timespan, xp);
    xp = Xout(end,:)';
    Traj = [Traj; Tout(:), Xout, u_path];
    
    uf = u_path(end);
    ti = tf;
    tf = ti+dT;

end

figure
plot((Traj(:,1)),Traj(:,3),'b',flip(Traj(:,1)),r*ones(size(Traj(:,1))), 'r--','linewidth',3);
set(gca,'fontsize', 16);
xlabel('Time (s)');
legend('y', 'r');

figure
plot((Traj(:,1)), Traj(:,4), 'b', 'linewidth',3);
set(gca,'fontsize', 16);
xlabel('Time (s)');
legend('u');

%% Discrete Time Updates - Adding LQE

dT = .04;           % Sample rate (control loop time)
[Kd, Sd, ed] = lqrd(A,B,Q,R, dT);

G=eye(2,2);
Qe=diag([0 .1]);
Re=.1;
L=lqed(A,G,C,Qe,Re, dT);

F=inv(C*inv(-(A-B*Kd))*B);

u=0;
x = [[0; 5]; [0; 0]]; xc = [0;0]; r = 1;
ti = 0; tf = ti + dT; Traj = [];
% P = 1/s^2/(s+5); [A,B,C,D]=ssdata(P);
for k=1:1:5/dT

    xp=x(1:2);
    xe=x(3:4);
    x
    xp
    y=C*xp;
    ye=C*xe;

    u0 = u;
    u=F*r-Kd*xe;
    if abs(u) > 15
        u = (abs(u)/u)*15;
    end

    timespan = [ti:dT/10:tf]';
    u_path = u0 + ((timespan-ti)/(tf-ti))*u;
%     [Yout, Tout, Xout]=lsim(ss(A,B,C,D),u*ones(size(timespan)),timespan,xp);
    [Tout, Xout] = ode45(@(t,x) stabilize_pert_lqe(t,x, u0, u, timespan, dT, y, ye, L), timespan, x);
    x = Xout(end,:)'
    Traj = [Traj; Tout(:), Xout,u_path];
    
    ti = tf;
    tf = ti+dT;

end

figure
plot((Traj(:,1)),Traj(:,3),'b',flip(Traj(:,1)),r*ones(size(Traj(:,1))), 'r--','linewidth',3);
set(gca,'fontsize', 16);
xlabel('Time (s)');
legend('y', 'r');

figure
plot((Traj(:,1)), Traj(:,4), 'b', 'linewidth',3);
set(gca,'fontsize', 16);
xlabel('Time (s)');
legend('u');