function [xdot] = stabilize(t, x)
    %% Defining Constants
    Ix = 10;                 % rotational moment of inertia
    Iy = 10;                 % rotational moment of inertia
    Iz = 10;                 % rotational moment of inertia
    m = 10;                 % mass of the rocket in kg
    C_l_prime = .01;        % slope of cl vs alpha curve
    C_d_fit = [1.294e-4 1.0734e-5 6.972e-3];% quadratic fit coefficients for C_d vs alpha
    rho = 1.17;             % density in kg/m^3
    S = .01;               % wing area in m^2
    area = .0001;              % wing cross-sectional area
    d_l = 0.01;             % distance from roll axis to aerodynamic center
    
    v = 70;                 % Air Speed (m/s)
    C_l
    
    sym p q r
    
    A = [
        0 -r*(Iz - Iy)/Ix -q(Iz - Iy)/Ix;
        -r*(Ix - Iz)/Iy 0 -p*(Ix-Iz)/Iy;
        -q*(Iy-Ix)/Iz -p(Iy-Ix)/Iz 0;
    ];
    B = [
        1.5*rho*v^2*Cl*S*d_l/Ix;
        0;
        0;
    ];

    A_lin = jacobian(A,[p q r])

end