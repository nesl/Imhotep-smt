close all;
clear all;
clc;
addpath('utils');
imhotepSMTPath = '../../';  %path to Imhotep-SMT
addpath(imhotepSMTPath);	

% flag
noise_flag        = false;
attack_flag       = true;
SMT_flag          = true;

%% Initalize the SMT solver
global smt uav F M

F                       = 0;       % Force in z direction
M                       = [0;0;0]; % 3D moments

smt                     = ImhotepSMT();
uav                     = nlsys();
n                       = 13; 
p                       = 12; 
m                       = 4; 
tau                     = 2;
h                       = @(s) [ s(1:3); s(4:6); RotToRPY_ZXY_wrapper(QuatToRot(s(7:10))); s(11:13) 
                                %{(x,y,z), (v_x,v_y,v_z), (phi, theta, yaw), (omega_phi, omega_theta, omega_yaw)
                        ];
f                       = @(s,u) [];                    
uav.init(f, h, @observer_uav, [], n, p, m, tau);

max_sensors_under_attack= int8(1); % maximum sensors under attack
safe_sensors            = [1,2,3,7,8,9,10,11,12];
noise_bound             = [0; 0; 0; 0.1*ones(3,1); 0; 0; 0; 0*ones(3,1)];     

smt.initNonLinearSolver(uav, max_sensors_under_attack, safe_sensors, noise_bound);


%% Loading Waypts
disp('Loading Waypts ...');
waypts = def_waypts();

map = {};
start = {waypts(1,:)};
stop  = {waypts(end,:)};
path{1} = waypts;

%% Generate trajectory
disp('Generating Trajectory ...');
trajectory_generator([], [], map, path);

%% Run trajectory
trajectory = test_trajectory(start, stop, map, path, noise_flag, attack_flag, SMT_flag); % with visualization
