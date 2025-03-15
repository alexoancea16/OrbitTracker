function dx = fct_rk(t, x, k)
    % componentele vectorului de stare
    r1 = x(1:3);
    r1_dot = x(4:6);
    r2 = x(7:9);
    r2_dot = x(10:12);
    epsilon = x(13);
    
    % constante
    G = 6.674 * 10^-11;
    M_earth = 5.972 * 10^24;
    R_earth = 6371000;
    J2 = 1.08262668 * 10^-3;
    omega_earth = 7.2921 * 10^-5;
    
    r1_norm = norm(r1); 
    r2_norm = norm(r2);

    g_grav_1 = -G * M_earth * r1/ r1_norm^3;

    z2_1 = r1(3)^2;
    factor_J2_1 = -(3/2) * J2 * G * M_earth * R_earth^2 / r1_norm^5;
    g_J2_1 = factor_J2_1 * [
        r1(1) * (1 - 5 * z2_1 / r1_norm^2);
        r1(2) * (1 - 5 * z2_1 / r1_norm^2);
        r1(3) * (3 - 5 * z2_1 / r1_norm^2)
    ];

    g_centrifugal_1 = omega_earth^2 * [r1(1); r1(2); 0];

    g_coriolis_1 = 2 * omega_earth * [
        r1_dot(2);
        -r1_dot(1);
        0
    ];

    g_out_1 = g_grav_1 + g_J2_1 + g_centrifugal_1 + g_coriolis_1;

    g_grav_2 = -G * M_earth * r2/ r2_norm^3;

    z2_2 = r2(3)^2;
    factor_J2_2 = -(3/2) * J2 * G * M_earth * R_earth^2 / r2_norm^5;
    g_J2_2 = factor_J2_2 * [
        r2(1) * (1 - 5 * z2_2 / r2_norm^2);
        r2(2) * (1 - 5 * z2_2 / r2_norm^2);
        r2(3) * (3 - 5 * z2_2 / r2_norm^2)
    ];

    g_centrifugal_2 = omega_earth^2 * [r2(1); r2(2); 0];

    g_coriolis_2 = 2 * omega_earth * [
        r2_dot(2);
        -r2_dot(1);
        0
    ];

    g_out_2 = g_grav_2 + g_J2_2 + g_centrifugal_2 + g_coriolis_2;
    
    % epsilon
    epsilon_dot = norm(r1 - r2)^2 / r2_norm^2;
    
    u = @(t) k * 1e-3 * double(t>=0);

    % derivata vectorului de stare
    dx = zeros(13, 1);
    dx(1:3) = r1_dot;
    dx(4:6) = g_out_1 + u(t);
    dx(7:9) = r2_dot;
    dx(10:12) = g_out_2;
    dx(13) = epsilon_dot;
end