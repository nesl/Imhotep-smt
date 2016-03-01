function params = nanoplus()
% NANOPLUS basic parameters for nanoplus quadrotor
%   nanoplus outputs the basic parameters (mass, gravity and inertia) of
%   the quadrotor nanoplus

m = 0.42; %kg
g = 9.81;
% I = [8.795203e-3,   6.39935e-4,        2.5927e-5;
%      6.39935e-4,    5.147140e-3,       7.203e-6;
%      2.5927e-5,     7.203e-6,          1.3625726e-2];

I = [2.04016e-5,     0,                0;
     0,              1.56771e-5,       0;
     0,              0,                3.51779e-5];

params.mass = m;
params.I    = I;
params.invI = inv(I);
params.grav = g;
params.arm_length = 0.086;

params.arm_mat = [0.12586, 0.12586;
                  0.12586, 0.12586;
                  0.12586, 0.12586;
                  0.12586, 0.12586]; 
              
params.kf = 1.5693e-7;
params.km = 2.38e-9;
% l_1x l_1y
% l_2x l_2y
% l_3x l_3y
% l_4x l_4y

Ixx = I(1,1);
Iyy = I(2,2);
Izz = I(3,3);

params.maxangle = 40 * pi / 180; %you can specify the maximum commanded angle here
params.maxF     = 2.5 * m * g;
params.minF     = 0.05 * m * g;

%F = param.kforce * omega^2
%M = params.kmoment * omega^2
%armlength is the distance from the center of the craft to the prop center
% params.kforce = 6.11e-8; %in Newton/rpm^2
% params.kmoment = 1.5e-9; %in Newton*meter/rpm^2
% params.armlength = 0.0849; %in meters
%
% params.FM_omega2 = [params.kforce,params.kforce,params.kforce,params.kforce;...
%     0,params.armlength*params.kforce,0,-params.armlength*params.kforce;...
%     -params.armlength*params.kforce,0,params.armlength*params.kforce,0;...
%     params.kmoment,-params.kmoment,params.kmoment,-params.kmoment];
%
% params.omega2_FM = inv(params.FM_omega2);
%
% params.maxomega = sqrt(params.maxF/(4*params.kforce));
% params.minomega = sqrt(params.minF/(4*params.kforce));

% ***************** Controller PID values  **************************

params.kp_z = 50;
params.kd_z = 10;

params.kp_xy = 10;
params.kd_xy = 5;


t_attitude  = 0.15; %rise time of the attitude controller
xi_attitude = 1; %damping ratio for attitude controller

params.kp_roll = 3.24*Ixx/t_attitude^2;
params.kd_roll = 3.6*xi_attitude*Ixx/t_attitude;

params.kp_pitch = 3.24*Iyy/t_attitude^2;
params.kd_pitch = 3.6*xi_attitude*Ixx/t_attitude;

t_yaw  = 0.2; %rime time of yaw controller
xi_yaw = 1.0; %damping ratio for yaw controller
params.kp_yaw = 3.24*Izz/t_yaw^2;
params.kd_yaw = 3.6*xi_yaw*Izz/t_yaw;


params.ki_xy = .03; %integral gains
params.ki_z  = .02;


end
