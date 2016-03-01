%% Housekeeping
clc; close all; clear all;  rand('state',0);    randn('state',0);
imhotepSMTPath = '../../';  %path to Imhotep-SMT
addpath(imhotepSMTPath);	


% factor = 7;
% n                   = 10*factor;
% p                   = 35*factor;
% s_bar               = 10*factor;

n                   = 2;
p                   = 350;
s_bar               = 40;

m                   = 1;


l                   = m + p;
safe_sensors            = [];

A                   = randn(n,n);
A                   = A/(max(svd(p)) + 0.1);
C                   = randn(p,n);
B                   = randn(n,m);

sys                         = ss(A,B,C,0, 0.01);  % create the sysetm

max_sensors_under_attack= int8(s_bar); % maximum sensors under attack
%noise_bound             = zeros(p,1);     
noise_bound             = ones(p,1);     
x0                  = randn(n,1);

%% attacker
per                         = randperm(p-1); % the attacker randmoizes the sensor indecies
attacked_sensor_index       = per(1:s_bar); % the attacker then picks the first 14 sensors to attack them
attackpower                 = 50;


noise_power                 = 0.1;
process_noise_power         = 0.5;


%% Initialize Imhotep-SMT solver



smt = ImhotepSMT();
smt.init(sys, max_sensors_under_attack, safe_sensors, noise_bound );

x = x0;
for t = 1 : n
    % load the outputs from collected data
    y                                       = C*x + noise_power*rand(p,1);
    y(attacked_sensor_index)                = y(attacked_sensor_index) + attackpower*randn();

    % the system has no inputs. Set the input signal u to zero
    u                                       = zeros(size(B,2),1);
    
    tic;
    [xhat, b] = smt.addInputsOutputs(u, y);
    time = toc;
    disp(['Execution time = ' num2str(time) ' sec']);
    time_elapsed(t) = time;
    
    x                                       = A * x + process_noise_power*rand(n,1);
end