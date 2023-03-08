function test_template()
    a = arduino('COM3','Mega2560','Libraries','I2C','ForceBuildOn',true);

    % Servo initialization
    f1 = servo(a, 'D3');        % TODO: Change these pins
    f2 = servo(a, 'D3');
    f3 = servo(a, 'D3');

    p1 = servo(a, 'D3');

    % IMU initialization
    sample_rate = 100;
    samples_per_read = 10;

    imu = mpu6050(a, 'Sample rate', sample_rate, 'SamplesPerRead', samples_per_read, 'OutputFormat', 'matrix');

    % Kill Switch from receiver
    run = 1;

    % Control parameters
    Ix = .1;                % rotational moment of inertia
    m = 10;                 % mass of the rocket in kg
    C_l_prime = .01;        % slope of cl vs alpha curve
    rho = 1.17;             % density in kg/m^3
    S = .01;                % wing area in m^2
    area = .0001;              % wing cross-sectional area
    d_l = 0.15;             % distance from roll axis to aerodynamic center
    imu_offset = 0.01;
    g = 9.8066;
    dT = .04;              % Sample rate (control loop time)
    
    v = 70;                 % Air Speed (m/s) TODO: CHANGE THIS FOR DIFFERENT TUNNEL SPEEDS
    
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
    
    q1 = 5;
    r1 = 1;
    
    Q = C'*q1*C;
    R = r1;

    [Kd, Sd, ed] = lqrd(A,B,Q,R, dT);

    while(run)
        % Start timer to check duration of control loop
        start_time = tic;

        % Read IMU and Calculate State

        % Read
        [accel, angular_vel] = read(imu);
        a_x = accel(1);
        a_y = accel(2);
        a_z = accel(3);
        p = angular_vel(1);
        q = angular_vel(2);
        r = angular_vel(3);

        % Calcualte adjusted acceleration TODO: choose different based on
        % orientation
        radial = p^2*imu_offset;
        a_y = a_y - radial;
        theta = asind(a_y/g);

        % Write current values to the SD card (or outsource to DAS arduino)
        
        xp = [theta; r]; % Plant state
        % Currently: unknown orientation of IMU in housing, structure
        % should be correct

        % Calculate desired servo value
        u = -Kd*xp;
        if abs(u) > 15
            u = (abs(u)/u)*15;
        end
        cmd = (90+u)/180;

        % Set servo angles
        writePosition(f1, cmd);
        writePosition(f2, cmd);
        writePosition(f3, cmd);

        % Set perturbation servo angle (if required)


        end_time = toc;
        % Pause for the remaining time to enforce control loop frequency
        pause(dT - (end_time-start_time));
    end
    
end

