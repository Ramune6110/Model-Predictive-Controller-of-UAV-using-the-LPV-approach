function [] = DrowResults(X_ref, X_dot_ref, Y_ref, Y_dot_ref, Z_ref, Z_dot_ref, states_total, velocityXYZ_total, ref_angles_total, UTotal, plotl, t, t_angles)
    constants = initial_constants();
    innerDyn_length = constants{18}; % Number of inner control loop iterations
    
    %% Plot the trajectory
    % Trajectory
    figure(1);
    plot3(X_ref(:, 2), Y_ref(:, 2), Z_ref(:, 2), '-b', 'LineWidth', 2);
    hold on
    plot3(states_total(1:innerDyn_length:end, 7), states_total(1:innerDyn_length:end, 8), states_total(1:innerDyn_length:end, 9), 'r', 'LineWidth', 1);
    grid on;
    xlabel('X [m]', 'FontSize', 15)
    ylabel('Y [m]', 'FontSize', 15)
    zlabel('Z [m]', 'FontSize', 15)
    legend('position reference', 'controller', 'Location', 'best')

    %% Plot the positions and velocities individually
    % X_ref, X_dot_ref
    figure(2);
    subplot(2, 1, 1);
    plot(t(1:plotl), X_ref(1:plotl, 2), '-b', 'LineWidth', 2)
    hold on
    plot(t(1:plotl), states_total(1:innerDyn_length:end, 7), 'r', 'LineWidth', 1)
    grid on
    xlabel('Time [s]', 'FontSize', 15)
    ylabel('X [m]', 'FontSize', 15)
    legend('X position reference', 'controller', 'Location', 'best')

    subplot(2, 1, 2);
    plot(t(1:plotl), X_dot_ref(1:plotl, 2), '-b', 'LineWidth', 2)
    hold on
    plot(t(1:plotl), velocityXYZ_total(1:innerDyn_length:end, 1), 'r', 'LineWidth', 1)
    grid on
    xlabel('Time [s]', 'FontSize', 15)
    ylabel('X-Velocity [m/s]', 'FontSize', 15)
    legend('X velocity reference', 'controller', 'Location', 'best')

    % Y_ref, Y_dot_ref
    figure(3);
    subplot(2, 1, 1);
    plot(t(1:plotl), Y_ref(1:plotl, 2), '-b', 'LineWidth', 2)
    hold on
    plot(t(1:plotl), states_total(1:innerDyn_length:end, 8), 'r', 'LineWidth', 1)
    grid on
    xlabel('Time [s]', 'FontSize', 15)
    ylabel('Y [m]', 'FontSize', 15)
    legend('Y position reference', 'controller', 'Location', 'best')

    subplot(2, 1, 2);
    plot(t(1:plotl), Y_dot_ref(1:plotl, 2), '-b', 'LineWidth', 2)
    hold on
    plot(t(1:plotl), velocityXYZ_total(1:innerDyn_length:end, 2), 'r', 'LineWidth', 1)
    grid on
    xlabel('Time [s]', 'FontSize', 15)
    ylabel('Y-Velocity [m/s]', 'FontSize', 15)
    legend('Y velocity reference', 'controller', 'Location', 'best')

    % Z_ref, Z_dot_ref
    figure(4);
    subplot(2, 1, 1);
    plot(t(1:plotl), Z_ref(1:plotl, 2), '-b', 'LineWidth', 2)
    hold on
    plot(t(1:plotl), states_total(1:innerDyn_length:end, 9), 'r', 'LineWidth', 1)
    grid on
    xlabel('Time [s]', 'FontSize', 15)
    ylabel('Z [m]', 'FontSize', 15)
    legend('Z position reference', 'controller', 'Location', 'best')

    subplot(2, 1, 2);
    plot(t(1:plotl), Z_dot_ref(1:plotl, 2), '-b', 'LineWidth', 2)
    hold on
    plot(t(1:plotl), velocityXYZ_total(1:innerDyn_length:end, 3), 'r', 'LineWidth', 1)
    grid on
    xlabel('Time [s]', 'FontSize', 15)
    ylabel('Z-Velocity [m/s]', 'FontSize', 15)
    legend('Z velocity reference', 'controller', 'Location', 'best')

    %% Plot the angles individually
    figure(5);
    % Phi
    subplot(3, 1, 1);
    plot(t_angles(1:length(ref_angles_total(:, 1))), ref_angles_total(:, 1), '-b', 'LineWidth', 2)
    hold on
    plot(t_angles(1:length(states_total(:, 10))), states_total(:, 10), 'r', 'LineWidth', 1)
    grid on
    xlabel('Time [s]', 'FontSize', 15)
    ylabel('Phi-angle [rad]', 'FontSize', 15)
    legend('Phi angle reference', 'controller', 'Location', 'best')
    % Theta
    subplot(3, 1, 2);
    plot(t_angles(1:length(ref_angles_total(:, 2))), ref_angles_total(:, 2), '-b', 'LineWidth', 2)
    hold on
    plot(t_angles(1:length(states_total(:, 11))), states_total(:, 11), 'r', 'LineWidth', 1)
    grid on
    xlabel('Time [s]', 'FontSize', 15)
    ylabel('Theta-angle [rad]', 'FontSize', 15)
    legend('Theta angle reference', 'controller', 'Location', 'best')
    % Psi
    subplot(3, 1, 3);
    plot(t_angles(1:length(ref_angles_total(:, 3))), ref_angles_total(:, 3), '-b', 'LineWidth', 2)
    hold on
    plot(t_angles(1:length(states_total(:, 12))), states_total(:, 12), 'r', 'LineWidth', 1)
    grid on
    xlabel('Time [s]', 'FontSize', 15)
    ylabel('Psi-angle [rad]', 'FontSize', 15)
    legend('Psi angle reference', 'controller', 'Location', 'best')

    %% Plot the inputs
    figure(6);
    % U1
    subplot(4, 1, 1);
    plot(t_angles(1:length(states_total(:, 10))), UTotal(:, 1), 'b', 'LineWidth', 2)
    grid on
    ylabel('U1 [N]', 'FontSize', 15)
    % U2
    subplot(4, 1, 2);
    plot(t_angles(1:length(states_total(:, 10))), UTotal(:, 2), 'b', 'LineWidth', 2)
    grid on
    ylabel('U2 [Nm]', 'FontSize', 15)
    % U3
    subplot(4, 1, 3);
    plot(t_angles(1:length(states_total(:, 10))), UTotal(:, 3), 'b', 'LineWidth', 2)
    grid on
    ylabel('U3 [Nm]', 'FontSize', 15)
    % U4
    subplot(4, 1, 4);
    plot(t_angles(1:length(states_total(:, 10))), UTotal(:, 4), 'b', 'LineWidth', 2)
    grid on
    xlabel('Time [s]', 'FontSize', 15)
    ylabel('U4 [Nm]', 'FontSize', 15)
end