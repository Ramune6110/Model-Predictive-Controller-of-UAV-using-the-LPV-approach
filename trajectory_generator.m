function [X_ref, X_dot_ref, X_dot_dot_ref, Y_ref, Y_dot_ref, Y_dot_dot_ref, Z_ref, Z_dot_ref, Z_dot_dot_ref, psi_ref] = trajectory_generator(t, r, f, height_i, height_f)
    constants  = initial_constants();
    trajectory = constants{22}; 
    
    alpha = 2 * pi * f .* t;
    d_height = height_f - height_i;
    
    if trajectory == 1
        % python version trajectory = 1
        x = r .* cos(alpha);
        y = r .* sin(alpha);
        z = height_i + d_height / t(end) * t;

        x_dot = -r .* sin(alpha) * 2 * pi * f;
        y_dot = r .* cos(alpha) * 2 * pi * f;
        z_dot = round(d_height / t(end) * ones(length(t)), 8);

        x_dot_dot = -r .* cos(alpha) * (2 * pi * f)^2;
        y_dot_dot = -r .* sin(alpha) * (2 * pi * f)^2;
        z_dot_dot = 0 .* ones(length(t));
    elseif trajectory == 2
        % python version trajectory = 8
        x = r / 5 .* sin(alpha) + t / 100;
        y = t / 100 - 1;
        z = height_i + d_height / t(end) * t;

        x_dot = r / 5 .* cos(alpha) * 2 * pi * f + 1 / 100;
        y_dot = 1 / 100 .* ones(length(t));
        z_dot = round(d_height / t(end) * ones(length(t)), 8);

        x_dot_dot = -r / 5 .* sin(alpha) * (2 * pi * f)^2;
        y_dot_dot = 0 .* ones(length(t));
        z_dot_dot = 0 .* ones(length(t));
    else 
        disp('No trajectory');
    end
    
    dx = [x(2) - x(1), x(2 : end) - x(1 : end - 1)];
    dy = [y(2) - y(1), y(2 : end) - y(1 : end - 1)];
    dz = [z(2) - z(1), z(2 : end) - z(1 : end - 1)];
    
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
    X_dot_dot_ref = [t' x_dot_dot'];
    Y_ref     = [t' y'];
    Y_dot_ref = [t' y_dot'];
    Y_dot_dot_ref = [t' y_dot_dot'];
    Z_ref     = [t' z'];
    Z_dot_ref = [t' z_dot'];
    Z_dot_dot_ref = [t' z_dot_dot'];
    psi_ref   = [t' psi'];
end
