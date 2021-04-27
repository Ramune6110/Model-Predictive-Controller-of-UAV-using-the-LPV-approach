function [X_ref, X_dot_ref, Y_ref, Y_dot_ref, Z_ref, Z_dot_ref, psi_ref] = trajectory_generator(t, r, f, height_i, height_f)
    constants = initial_constants();
    Ts = constants{7};
    innerDyn_length = constants{18}; % Number of inner control loop iterations
    
    alpha = 2 * pi * f .* t;
    d_height = height_f - height_i;
    
    x = r .* cos(alpha);
    y = r .* sin(alpha);
    z = height_i + d_height / t(end) * t;
    
    dx = [x(2) - x(1), x(2 : end) - x(1:end - 1)];
    dy = [y(2) - y(1), y(2 : end) - y(1 : end - 1)];
    dz = [z(2) - z(1), z(2 : end) - z(1 : end - 1)];

    x_dot = dx .* (1 / (Ts * innerDyn_length));
    y_dot = dy .* (1 / (Ts * innerDyn_length));
    z_dot = round(dz .* (1 / (Ts * innerDyn_length)), 8);
   
    psi = zeros(1, length(x));
    psi(1) = atan2(y(1), x(1)) + pi / 2;
    psi(2 : end) = atan2(dy(2 : end), dx(2 : end));
    
    for i = 1:length(psi)
        if psi(i) < 0
            psi(i) = 2 * pi - abs(psi(i));
        end
    end
   
    for i = 1:length(psi)
        if i > 1
            if abs(psi(i) - psi(i - 1)) > pi
                psi(i : end) = psi(i : end) + 2 * pi;
            end
        end
    end
    
    % reference
    X_ref     = [t' x'];
    X_dot_ref = [t' x_dot'];
    Y_ref     = [t' y'];
    Y_dot_ref = [t' y_dot'];
    Z_ref     = [t' z'];
    Z_dot_ref = [t' z_dot'];
    psi_ref   = [t' psi'];
end
