clear
close all
clc

%% Main file for controllering the drone

%% The controller consists of the position controller
%% State feedback linearization

%% The relevant function files to this main file are the following:
   % initial_constants.m
   % LPV__cont_discreate.m
   % MPC_simplification.m
   % nonlinear_drone_model.m
   % trajectory_generator.m
   % pos_controller.m
   
%% Load the cons tant values
constants = initial_constants();
Ts = constants{7};
controlled_states = constants{14}; % Number of controlled states
innerDyn_length = constants{18}; % Number of inner control loop iterations

%% Generate the reference signals
t = 0 : Ts * innerDyn_length : 100;
t_angles = (0 : Ts : t(end))';
r = 2;
f = 0.025;
height_i = 2;
height_f = 5;
[X_ref, X_dot_ref, Y_ref, Y_dot_ref, Z_ref, Z_dot_ref, psi_ref] = trajectory_generator(t, r, f, height_i, height_f);

poltl = length(t); % Number of outer control loop iterations

%% Load the initial state vector

ut = 0;
vt = 0;
wt = 0;
pt = 0;
qt = 0;
rt = 0;
xt = 0;  % X_ref(1, 2) Initial translational position
yt = -1; % Y_ref(1, 2) Initial translational position
zt = 0;  % Z_ref(1, 2) Initial translational position
phit = 0;             % Initial angular position
thetat = 0;           % Initial angular position
psit = psi_ref(1, 2); % Initial angular position

states = [ut, vt, wt, pt, qt, rt, xt, yt, zt, phit, thetat, psit];
states_total = states;

% Assume that first Phi_ref, Theta_ref, Psi_ref are equal to the first
% phit, thetat, psit
ref_angles_total  = [phit, thetat, psit];
velocityXYZ_total = [X_dot_ref(1, 2), Y_dot_ref(1, 2), Z_dot_ref(1, 2)];
