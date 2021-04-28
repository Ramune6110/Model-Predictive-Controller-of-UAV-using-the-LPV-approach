function [Phi_ref, Theta_ref, U1] = pos_controller(X_ref, X_dot_ref, X_dot_dot_ref, Y_ref, Y_dot_ref, Y_dot_dot_ref, Z_ref, Z_dot_ref, Z_dot_dot_ref, Psi_ref, states)
    constants = initial_constants();
    m = constants{4}; 
    g = constants{5};
    
    %% Assign the states
    % States : [u, v, w, p, q, r, x, y, z, phi, theta, psi]
    u     = states(1);
    v     = states(2);
    w     = states(3);
    x     = states(7);
    y     = states(8);
    z     = states(9);
    phi   = states(10);
    theta = states(11);
    psi   = states(12);
    
    % Rotational matrix that relates u, v, w with x_dot, y_dot, z_dot
    R_matrix = [cos(theta) * cos(psi), sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi), cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi);
                cos(theta) * cos(psi), sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi), cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi);
                -sin(theta), sin(phi) * cos(theta), cos(phi) * cos(theta)];
    
    x_dot = R_matrix(1, :) * [u; v; w];
    y_dot = R_matrix(2, :) * [u; v; w];
    z_dot = R_matrix(3, :) * [u; v; w];
    
    %% Compute the errors
     ex     = X_ref - x;
     ex_dot = X_dot_ref - x_dot;
     ey     = Y_ref - y;
     ey_dot = Y_dot_ref - y_dot;
     ez     = Z_ref - z;
     ez_dot = Z_dot_ref - z_dot;
     
     %% Compute the constants K1, K2, and the values vx, vy, vz to stabilize the position subsystem
     Ax = [0 1; 0 0];
     Bx = [0; 1];
     px = constants{19};
     Kx = place(Ax, Bx, px);
     ux = -Kx * [ex; ex_dot];
     vx = -ux;
     
     Ay = [0 1; 0 0];
     By = [0; 1];
     py = constants{20};
     Ky = place(Ay, By, py);
     uy = -Ky * [ey; ey_dot];
     vy = -uy;
     
     Az = [0 1; 0 0];
     Bz = [0; 1];
     pz = constants{21};
     Kz = place(Az, Bz, pz);
     uz = -Kz * [ez; ez_dot];
     vz = -uz;
     
%      % python version
%     kx1 = (px(1, 1) - (px(1, 1) + px(1, 2)) / 2)^2 - (px(1, 1) + px(1, 2))^2 / 4;
%     kx2 = px(1, 1) + px(1, 2);
%     kx1 = real(kx1);
%     kx2 = real(kx2);
%     
%     ky1 = (py(1, 1) - (py(1, 1) + py(1, 2)) / 2)^2 - (py(1, 1) + py(1, 2))^2 / 4;
%     ky2 = py(1, 1) + py(1, 2);
%     ky1 = real(ky1);
%     ky2 = real(ky2);
%     
%     kz1 = (pz(1, 1) - (pz(1, 1) + pz(1, 2)) / 2)^2 - (pz(1, 1) + pz(1, 2))^2 / 4;
%     kz2 = pz(1, 1) + pz(1, 2);
%     kz1 = real(kz1);
%     kz2 = real(kz2);
% 
%     % Compute the values vx, vy, vz for the position controller
%     ux = kx1 * ex + kx2 * ex_dot;
%     uy = ky1 * ey + ky2 * ey_dot;
%     uz = kz1 * ez + kz2 * ez_dot;
%     
%     vx = X_dot_dot_ref - ux(1, 1);
%     vy = Y_dot_dot_ref - uy(1, 1);
%     vz = Z_dot_dot_ref - uz(1, 1);
     
     %% Compute phi, theta, U1
     a = vx / (vz + g);
     b = vy / (vz + g);
     c = cos(Psi_ref);
     d = sin(Psi_ref);
     
     tan_theta = a * c + b * d;
     Theta_ref = atan(tan_theta);
     
     if or(abs(Psi_ref) < pi / 4, abs(Psi_ref) > 3 * pi / 4)
         tan_phi = cos(Theta_ref) * (tan(Theta_ref) * d - b) / c;
     else
         tan_phi = cos(Theta_ref) * (a - tan(Theta_ref) * c) / d;
     end
     
     Phi_ref = atan(tan_phi);
     U1 = (vz + g) * m / (cos(Phi_ref) * cos(Theta_ref));
    
end