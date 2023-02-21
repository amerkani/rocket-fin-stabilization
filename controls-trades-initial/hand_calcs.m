%% Defining Constants
I = 10;                 % rotational moment of inertia
m = 10;                 % mass of the rocket in kg
C_l_prime = .01;        % slope of cl vs alpha curve
C_d_fit = [1.294e-4 1.0734e-5 6.972e-3];% quadratic fit coefficients for C_d vs alpha
rho = 1.17;             % density in kg/m^3
S = .001;               % wing area in m^2
area = .0001;              % wing cross-sectional area
d_l = 0.01;             % distance from roll axis to aerodynamic center

syms alpha alpha_dot beta beta_dot v

C_l = C_l_prime*alpha
C_d = C_d_fit(1)*alpha.^2 + C_d_fit(2)*alpha + C_d_fit(3)

eqs = [
    beta_dot;
    3*0.5*C_l_prime*alpha*rho*v.^2*S*d_l./I;
    alpha_dot;
    0;
    -0.5*(C_d_fit(1)*alpha.^2 + C_d_fit(2)*alpha + C_d_fit(3))*rho*v.^2*A/m;
];
A = jacobian(eqs, [beta; beta_dot; alpha; alpha_dot; v])

%% Defining the State
function [xdot] = vehicle_dynamics(t,x,I,m,C_l_prime,C_d_fit,rho,S,A,d_l) %#ok<DEFNU> 

C_l = C_l_prime*x(3);
C_d = C_d_fit(0)*x(3)^2 + C_d_fit(2)*x(3) + C_d_fit(3);

xdot = [
    x(2);
    3*0.5*C_l*rho*x(5)^2*S*d_l/I;
    x(4);
    0;
    -0.5*C_d*rho*x(5)^2*A/m;
];
end
