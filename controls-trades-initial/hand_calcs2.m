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
C = [1 0 0];
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

q1 = 20;
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
[Kd, Sd, ed] = lqr(Ad,Bd,Q,R);

xp = [0; 1]; xc = [0;0]; r = 0;
ti = 0; tf = ti + dT; Traj = [];
% P = 1/s^2/(s+5); [A,B,C,D]=ssdata(P);
for k=1:1:30/dT

    u = Kd*xp;
    if abs(u) > 15
        u = (abs(u)/u)*15;
    end

    timespan = [ti:dT/10:tf]';
    [Yout, Tout, Xout]=lsim(ss(A,B,C,D),u*ones(size(timespan)),timespan,xp);
    xp = Xout(end,:)';
    Traj = [Traj; Tout(:), Xout,Yout(:),u*ones(size(timespan))];
    
    ti = tf;
    tf = ti+dT;

end

figure
plot(flip(Traj(:,1)),Traj(:,3),'b',flip(Traj(:,1)),r*ones(size(Traj(:,1))), 'r--','linewidth',3);
set(gca,'fontsize', 16);
xlabel('Time (s)');
legend('y', 'r');

figure
plot(flip(Traj(:,1)), Traj(:,5), 'b', 'linewidth',3);
set(gca,'fontsize', 16);
xlabel('Time (s)');
legend('u');