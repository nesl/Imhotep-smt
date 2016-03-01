%  * Copyright (c) 2015 The Regents of the University of California.
%  * All rights reserved.
%  *
%  * Redistribution and use in source and binary forms, with or without
%  * modification, are permitted provided that the following conditions
%  * are met:
%  * 1. Redistributions of source code must retain the above copyright
%  *    notice, this list of conditions and the following disclaimer.
%  * 2. Redistributions in binary form must reproduce the above
%  *    copyright notice, this list of conditions and the following
%  *    disclaimer in the documentation and/or other materials provided
%  *    with the distribution.
%  * 3. All advertising materials mentioning features or use of this
%  *    software must display the following acknowledgement:
%  *       This product includes software developed by Networked &
%  *       Embedded Systems Lab at UCLA
%  * 4. Neither the name of the University nor that of the Laboratory
%  *    may be used to endorse or promote products derived from this
%  *    software without specific prior written permission.
%  *
%  * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS''
%  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
%  * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
%  * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS
%  * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
%  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
%  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
%  * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
%  * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
%  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
%  * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
%  * SUCH DAMAGE.
%  */

%   This is an example of using Imhotep-SMT in estimating the state of an
%   Unmanned Ground Vehicle (UGV) while one of its sensors is under attack.  


%% Housekeeping
clc; close all; clear all;  rand('state',0);    randn('state',0);
imhotepSMTPath = '../../';  %path to Imhotep-SMT
addpath(imhotepSMTPath);	

%% Robot parameters

% Motor Parameters
Lm                      = 1E-3;         % Motor Inducatnce
Rm                      = 3.1;     
% Motor resistance
Jg                      = 1.2E-6;       % Angular mass of gears
Bg                      = 6.7E-7;       % Mechanical resistance of gears to rotate
g                       = 204;          % gear ratio
alpha                   = 1E-3;         % Current-Torque ratio of the Motor
radius                  = 0.015;        % radius of sprocket wheel
Kt                      = 1E-3;         % Compliance of the track

% Chasis Parameters
Mt                      = 0.8;          % Mass of the tank
Jt                      = 5E-4;         % Angular mass of the Tank
Width                   = 0.1;          % Width of the tank (B)
Br                      = 1;            % Mechanical friction of the tracks to roll
Bs                      = 14;           % Mechanical friction of the tracks to slide
Bl                      = 0.7;          % Mechanical friction of the tracks to turn
Sl                      = 0.3;          % Lateral friction of Tracks


%% Robot ODE describign the dynamics

f_cont                  = @(t,x)[  (1/Mt) * x(6) - (Bl/Mt) * x(1);  %   \dot{v}     = (1/Mt) * F - (Bl/Mt) * v;
                            (1/Jt) * x(7) - (Br/Jt) * x(2); %   \dot{omega} = (1/Jg) * T - (Br/Jg) * omega;
                            x(1) * cos(x(5));               %   \dot{x}     =  v * cos(theta);
                            x(1) * sin(x(5));               %   \dot{y}     = v * sin(theta);
                            x(2);                           %   \dot{theta} = omega;
                            0;                              %   \dot{F}     = 0
                            0;                              %   \dot{T}     = 0
                        ];     % nonlinear state equations
            
h                       = @(x) [ x(3); x(4); x(2); x(1) + 0.5*Width*x(2); x(1) - 0.5*Width*x(2)
                        ];
%% Discritized dynamics
global Ts
Ts                      = 0.001;

Av                      = - (Bl/Mt);
Aw                      = - (Br/Jt);

Bv                      = (1/Mt);
Bw                      = (1/Jt);

Av_d                    = exp(Av*Ts);
Aw_d                    = exp(Aw*Ts);

Bv_d                    = (Bv/Av)*(Av_d - 1);
Bw_d                    = (Bw/Aw)*(Aw_d - 1);
        
f_disc                  = @(x,u)[   Av_d * x(1)     + Bv_d * u(1);                  % v(t+1)     = Av_d * v(t)     + Bv_d * F(t)
                            Aw_d * x(2)     + Bw_d * u(2);                          % omega(t+1) = Aw_d * omega(t) + Bw_d * T(t)
                            x(3) + (x(1)/x(2)) *(sin(x(5) + Ts * x(2)) - sin(x(5)));% x(t+1)     = x(t) + (v(t)/omega(t)) * (sin(theta(t) + Ts omega(t)) - sin(theta(t)) 
                            x(4) - (x(1)/x(2)) *(cos(x(5) + Ts * x(2)) - cos(x(5)));% y(t+1)     = y(t) - (v(t)/omega(t)) * (cos(theta(t) + Ts omega(t)) - cos(theta(t)) 
                            x(5) + Ts*x(2);                                         % theta(t+1) = theta(t) + Ts * omega(t)
                        ];

%% Initalize the SMT solver
smt                     = ImhotepSMT();
ugv                     = nlsys();
n                       = 5; 
p                       = 5; 
m                       = 2; 
tau                     = 3;
ugv.init(f_disc, h, @observer_ugv, [], n, p, m, tau);

max_sensors_under_attack= int8(1); % maximum sensors under attack
safe_sensors            = [1,2];
noise_bound             = [0.5; 0.5; 0.5; 0.5; 0.5];      

smt.initNonLinearSolver(ugv, max_sensors_under_attack, safe_sensors, noise_bound);

%% Design controller

% rotation dynamics state = [theta omega]
A_rot                   = [0 1; 0 Aw];
B_rot                   = [0; Bw];

sys_rot                 = ss(A_rot, B_rot, [1 0], 0);
sys_rot_disc            = c2d(sys_rot, Ts);

% Q = [1 0 0; 0 1 0; 0 0 1000];   R        = eye(1);
% [K_lqr_rot,S,e]         = lqi(sys_rot_disc,Q,R,zeros(1)); % gain normal controller for in order



% %% prepare the 3D model
% 
% world                           = vrworld('ugv_vr2.wrl');
% open(world);
% 
% if(exist('fig') == 0)
%     fig                         = view(world, '-internal');
% end
% vrdrawnow;
% 
% 


%% Simulate the system
x0                      = [0 ;0; 0; 1; 0]; % set initial condition
x_cont                  = x0;
x_disc                  = x0;
X_cont                  = [];%x0;
X_hat                   = [];
Y                       = [];

x_i                     = zeros(1,1); %integrator states

theta_path              = pi/2;
radius_path             = 1;

u                       = [0; 0];
U                       = u;
NSIM                    = 500;
end_sim                 = 0;

figure
h1  = plot(x0(3,:), x0(4,:), 'r');  
title('X-Y','FontSize',18);

figure;
h2  = plot(U(2,:), 'r');  
title('Torque','FontSize',18);

for t = 0 : 550
%while(end_sim == 0)
    
    % Set the input
    
    
    theta_path          = theta_path + t*(2*pi/(NSIM*100));
    
    [x_path,y_path]     = pol2cart(theta_path, radius_path);
    % Solve the ODE for the continuous dynamics
    [~,x_cont]          = ode45( f_cont ,[0 Ts] ,[x_cont; u]);
    x_cont              = x_cont(end,1:5)';
    
    y                   = h(x_cont); %+ [0;0;0;50*randn(1); 0];
    Y                   = [Y y];
    X_cont              = [X_cont x_cont];
    [xhat, sensor_under_attack, status] = smt.addInputsOutputs(u, y);
    
%     if(t < tau)
%         xhat            = zeros(5,1);
%     elseif(status == -1)
%         xhat            = X_hat(:,end);
%     end
%     X_hat               = [X_hat xhat];
    
    error               = [x_path;y_path] - x_cont(3:4);
    if(norm(error) < 0.1)
        end_sim = 1;
    end
    
    
    [~, setpoint_r]     = cart2pol(error(1), error(2));
    setpoint_th         = atan2(error(2),error(1));
    
    err_theta           = [setpoint_th - x_cont(5)];
    x_i                 = x_i + Ts*err_theta;
%     if(x_i > pi)
%         x_i = x_i - 2*pi;
%     elseif(x_i < - pi)
%         x_i = x_i + 2*pi;
%     end
    %torque              = -K_lqr_rot*[atan2(sin(x_cont(5)), cos(x_cont(5)));x_cont(2);atan2(sin(x_i),cos(x_i))];
    torque              = [100 5 0.1]*[atan2(sin(err_theta),cos(err_theta)); atan2(sin(x_i),cos(x_i)); -x_cont(2)];
    force               = 10*setpoint_r;
    u                   = [force; torque];
    U                   = [U u];


    set(h1,'xData',X_cont(3,:));
    set(h1,'YData',X_cont(4,:));
    
    set(h2,'YData',U(2,:));
    drawnow;
    pause(0.01);
end

omega = [0.1*[0 : 550]', X_cont(2,:)'];
save('omega_noattack.txt', 'omega','-ascii','-double')

theta = [0.1*[0 : 550]', X_cont(5,:)'];
save('theta_noattack.txt', 'theta','-ascii','-double')


xy = [10*X_cont(3,:)', 10*X_cont(4,:)'];
save('xy_noattack.txt', 'xy','-ascii','-double')



%%
x0                      = [0 ;0; 0; 1; 0]; % set initial condition
x_cont                  = x0;
x_disc                  = x0;
X_cont                  = [];%x0;
X_hat                   = [];
Y                       = [];

x_i                     = zeros(1,1); %integrator states

theta_path              = pi/2;
radius_path             = 1;

u                       = [0; 0];
U                       = u;
NSIM                    = 500;
end_sim                 = 0;

attack_signal = 1000*ones(1,5000);

figure
h1  = plot(x0(3,:), x0(4,:), 'r');  
title('X-Y','FontSize',18);

figure;
h2  = plot(U(2,:), 'r');  
title('Torque','FontSize',18);

for t = 0 : 550
%while(end_sim == 0)
    
    % Set the input
    
    
    theta_path          = theta_path + t*(2*pi/(NSIM*100));
    
    [x_path,y_path]     = pol2cart(theta_path, radius_path);
    % Solve the ODE for the continuous dynamics
    [~,x_cont]          = ode45( f_cont ,[0 Ts] ,[x_cont; u]);
    x_cont              = x_cont(end,1:5)';
    
    y                   = h(x_cont) + [0;0;attack_signal(t+1);0; 0];
    Y                   = [Y y];
    X_cont              = [X_cont x_cont];
    [xhat, sensor_under_attack, status] = smt.addInputsOutputs(u, y);
    
    if(t < tau)
        xhat            = zeros(5,1);
    elseif(status == -1)
        xhat            = X_hat(:,end);
    end
    X_hat               = [X_hat xhat];
    
    error               = [x_path;y_path] - xhat(3:4);
    if(norm(error) < 0.1)
        end_sim = 1;
    end
    
    
    [~, setpoint_r]     = cart2pol(error(1), error(2));
    setpoint_th         = atan2(error(2),error(1));
    
    err_theta           = [setpoint_th - xhat(5)];
    x_i                 = x_i + Ts*err_theta;
%     if(x_i > pi)
%         x_i = x_i - 2*pi;
%     elseif(x_i < - pi)
%         x_i = x_i + 2*pi;
%     end
    %torque              = -K_lqr_rot*[atan2(sin(x_cont(5)), cos(x_cont(5)));x_cont(2);atan2(sin(x_i),cos(x_i))];
    torque              = [100 5 0.1]*[atan2(sin(err_theta),cos(err_theta)); atan2(sin(x_i),cos(x_i)); -xhat(2)];
    force               = 10*setpoint_r + 0.1*(0 - xhat(1));
    u                   = [force; torque];
    U                   = [U u];


    set(h1,'xData',X_cont(3,:));
    set(h1,'YData',X_cont(4,:));
    
    set(h2,'YData',U(2,:));
    drawnow;
    pause(0.01);
end


% close all
% state_idx = 5;
% plot([0:10000], X_cont(state_idx,:) - X_disc(state_idx,:));

figure; hold on;
title('X-Y')
plot(10*X_hat(3,4:end), 10*X_hat(4,4:end))
plot(X_cont(3,:), X_cont(4,:), 'r')

figure; hold on;
title('x');
%plot(X_hat(3,5:end)); hold on
plot(X_cont(3,5:end), 'r');

figure; hold on;
title('y');
%plot(X_hat(3,5:end)); hold on
plot(X_cont(4,5:end), 'r');

figure; hold on;
title('Theta')
plot(X_cont(5,5:end), 'r');

omega = [0.1*[0 : 550]', X_cont(2,:)'];
save('omega_attack_SMT.txt', 'omega','-ascii','-double')

theta = [0.1*[0 : 550]', X_cont(5,:)'];
save('theta_attack_SMT.txt', 'theta','-ascii','-double')

omega_hat = [0.1*[0 : 550]', X_hat(2,:)'];
save('omega_hat_attack_SMT.txt', 'omega_hat','-ascii','-double')

theta_hat = [0.1*[0 : 550]', X_hat(5,:)'];
save('theta_hat_attack_SMT.txt', 'theta_hat','-ascii','-double')


xy = [10*X_cont(3,:)', 10*X_cont(4,:)'];
save('xy_attack_SMT.txt', 'xy','-ascii','-double')

%% normal observer

x0                      = [0 ;0; 0; 1; 0]; % set initial condition
x_cont                  = x0;
x_disc                  = x0;
X_cont                  = [];%x0;
X_hat                   = [];
Y                       = [];

x_i                     = zeros(1,1); %integrator states

theta_path              = pi/2;
radius_path             = 1;

u                       = [0; 0];
U                       = u;
NSIM                    = 500;
end_sim                 = 0;

%attack_signal = -1000*sin([0 : 630]);
attack_signal = -1000*ones(1,632);%([0 : 630]);

figure
h1  = plot(x0(3,:), x0(4,:), 'r');  
title('X-Y','FontSize',18);

figure;
h2  = plot(U(2,:), 'r');  
title('Torque','FontSize',18);

for t = 0 : 550
%while(end_sim == 0)
    
    % Set the input
    
    
    theta_path          = theta_path + t*(2*pi/(NSIM*100));
    
    [x_path,y_path]     = pol2cart(theta_path, radius_path);
    % Solve the ODE for the continuous dynamics
    [~,x_cont]          = ode45( f_cont ,[0 Ts] ,[x_cont; u]);
    x_cont              = x_cont(end,1:5)';
    
    y                   = h(x_cont) + [0;0;attack_signal(t+1);0; 0];
    Y                   = [Y y];
    X_cont              = [X_cont x_cont];
    %[xhat, sensor_under_attack, status] = smt.addInputsOutputs(u, y);
    xhat(3)             = y(1);
    xhat(4)             = y(2);
    xhat(2)             = y(3);
    xhat(1)             = y(4) - 0.5*Width*xhat(2);
    xhat(5)             = x_cont(5);
    
    if(t < tau)
        xhat            = zeros(5,1);
    elseif(status == -1)
        xhat            = X_hat(:,end);
    end
    X_hat               = [X_hat xhat];
    
    error               = [x_path;y_path] - xhat(3:4);
    if(norm(error) < 0.1)
        end_sim = 1;
    end
    
    
    [~, setpoint_r]     = cart2pol(error(1), error(2));
    setpoint_th         = atan2(error(2),error(1));
    
    err_theta           = [setpoint_th - xhat(5)];
    x_i                 = x_i + Ts*err_theta;
%     if(x_i > pi)
%         x_i = x_i - 2*pi;
%     elseif(x_i < - pi)
%         x_i = x_i + 2*pi;
%     end
    %torque              = -K_lqr_rot*[atan2(sin(x_cont(5)), cos(x_cont(5)));x_cont(2);atan2(sin(x_i),cos(x_i))];
    torque              = [100 5 0.1]*[atan2(sin(err_theta),cos(err_theta)); atan2(sin(x_i),cos(x_i)); -xhat(2)];
    force               = 10*setpoint_r + 0.1*[0 - xhat(1)];
    u                   = [force; torque];
    U                   = [U u];


    set(h1,'xData',X_cont(3,:));
    set(h1,'YData',X_cont(4,:));
    
    set(h2,'YData',U(2,:));
    drawnow;
    pause(0.01);
end


xy = [10*X_cont(3,:)', 10*X_cont(4,:)'];
save('xy_attack_noSMT.txt', 'xy','-ascii','-double')

omega = [0.1*[0 : 550]', X_cont(2,:)'];
save('omega_attack_noSMT.txt', 'omega','-ascii','-double')

theta = [0.1*[0 : 550]', X_cont(5,:)'];
save('theta_attack_noSMT.txt', 'theta','-ascii','-double')

omega_hat = [0.1*[0 : 550]', X_hat(2,:)'+990];
save('omega_hat_attack_noSMT.txt', 'omega_hat','-ascii','-double')

theta_hat = [0.1*[0 : 550]', X_hat(5,:)'];
save('theta_hat_attack_noSMT.txt', 'theta_hat','-ascii','-double')