function [xdot] = stabilize(t, x, u0, uf, timespan, dT, y, ye, L)
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

    K = [.4721 5.1898];
  
    xp = x(1:2);
    xe = x(3:4);

%     u = -K*x;
%     if abs(u) > 15
%         u = (abs(u)/u)*15;
%     end
    ti = timespan(1);
    tf = timespan(end);
    u = u0 + ((t-ti)/(tf-ti))*uf;
%     tau_pert = .5*rho*v^2*C_l_prime*20*sin(t/dT)*(S/2)*d_l*(1/Ix);
    tau_pert = .5*rho*v^2*C_l_prime*.2*(S/2)*d_l*(1/Ix);
%     tau_pert = 0;

    A = [
        0 1;
        0 0;
    ];
    B = [
        0;
        1.5*rho*v^2*C_l_prime*S*d_l*(1/Ix)
    ];

    xpdot = A*xp + B*u + [0; tau_pert];
    xedot=A*xe+B*u+L*(y-ye);

    xdot=[xpdot(:,1); xedot(:,1)];
end