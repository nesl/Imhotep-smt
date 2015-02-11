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

%   In this example, we show how Imhotep-SMT can be used to securely
%   estimating the state of a driving car while performing active cruise
%   control. The simulation consists of two leader cars followed by two
%   followers cars. The follower cars are under sensor attacks. One of the
%   cars (blue) uese Imhotep-SMT to estimate the state while the other
%   follower car (red) uses standard least squares estimator. The
%   simulation uses a 3D virtual reality simulator to show the perfromance
%   of the two estimators. The simulation is currently restricted to the
%   case when the cars are driving in straight lines.

%   Author: Yasser Shoukry (yshoukry@ucla.edu)
%   Date: November 25, 2014

%% Book  keeping
clc; close all; clear all; rand('state',0); randn('state',0); 
imhotepSMTPath = '../../';  %path to Imhotep-SMT
addpath(imhotepSMTPath);	


%% car parameters and dynamics
Mt                              = 0.8;          % Mass of the tank
Br                              = 1;            % Mechanical friction of the tracks to roll

A                               = [0        1; 0    -Br/Mt];
B                               = [0         ; 1/Mt      ];
C                               = [1        0; 0        1;  0       1];   %GPS for x, Encoder for v
D                               = [0         ; 0         ;  0       ];
Ts                              = 0.01;
Sys                             = ss(A,B,C,D);
sys_car                         = c2d(Sys, Ts, 'ZOH');
O_car                           = obsv(sys_car.A, sys_car.C);

n                               = 2;      
p                               = 3;
%% Initalize the SMT solver
noise_power                     = 0.1;
noiseBound                      = ones(p,1)*noise_power;       % noise bounds
max_sensors_under_attack        = int8(1);         % maximum sensors under attack
safe_sensors                    = 1;

smt                             = ImhotepSMT();
smt.init(sys_car, max_sensors_under_attack, safe_sensors, 4*noiseBound );

%% Design Controller
Q = [500 0; 0 1];           R   = eye(1);
[K_lqr,S,e]                     = dlqr(sys_car.A,sys_car.B,Q,R,zeros(1)); % gain normal controller for in order


%% prepare the 3D model
world                           = vrworld('ugv_vr2.wrl');
open(world);

if(exist('fig') == 0)
    fig                         = view(world, '-internal');
end
vrdrawnow;

car                             = vrnode(world, 'Automobile2');
carCordInit                     = [13,0.5,-11];

car_SMT                         = vrnode(world, 'Automobile');

camera                          = vrnode(world, 'View1');
cameraCordInit                  = [2.4, 10.2, 25.8] - [-7.5, 0, 20];
diffCamera                      = cameraCordInit - carCordInit;


world2                          = vrworld('ugv_vr3.wrl');
open(world2);
if(exist('fig2') == 0)
    fig2                        = view(world2, '-internal');
end


car2                            = vrnode(world2, 'Automobile2');
car_noSMT                       = vrnode(world2, 'Automobile');

camera2                         = vrnode(world2, 'View1');
camera2CordInit                 = cameraCordInit;
diffCamera2                     = diffCamera;

%% Prepare the velocity plot
plot_length                     = 200;
velocity_SMT_history            = zeros(1,plot_length);
velocity_hat_SMT_history        = zeros(1,plot_length);

velocity_noSMT_history          = zeros(1,plot_length);
velocity_hat_noSMT_history      = zeros(1,plot_length);

attack_history                  = zeros(1,plot_length);

time                            = 0:Ts:(plot_length-1)*Ts;
figure;

subplot(3,1,1);     
h1 = plot(time, velocity_SMT_history, 'YDataSource', 'velocity_history'); 
hold on; 
h11 =plot(time, velocity_hat_SMT_history, 'b:','YDataSource', 'velocity_hat_history'); 
axis([0,2,0, 110])
title('Blue car: velocity (solid) and estimated velocity using Imhotep-SMT (dashed)','FontSize',18);

subplot(3,1,2);     
h2 = plot(time, velocity_noSMT_history, 'r');
hold on;
h22 = plot(time, velocity_hat_noSMT_history, 'r:');  
title('Red car: velocity (solid) and estimated velocity using least squares (dashed)','FontSize',18);
axis([0,2,0, 110])

subplot(3,1,3);             h3  = plot(time, attack_history, 'k');  
title('Attack signal','FontSize',18);
axis([0,2,-100, 100])

set(gcf,'currentcharacter','c')

%% initialize the car states
points                          = [46,        91,      137,  182];
angles                          = [-pi/2,     pi,     pi/2,  0];


initShift                       = 7;
state                           = [initShift+5;   0];
state_SMT                       = [2;   0];
state_noSMT                     = [2;   0];

angle                           = 0;
angle_noSMT                     = 0;

Fx                              = K_lqr  *([points(1); 5] - state);
Fx_SMT                          = K_lqr  *(state - [initShift;0] - state_SMT);
Fx_noSMT                        = K_lqr  *(state - [initShift;0] - state_noSMT);
end_sim                         = 0;

%% Simulate the cars
Y_bar_noSMT                     = zeros(p*n,1);
time_counter                    = 0;
attack_switch                   = 0;
stop_simulation                 = 0;
pause_sim                       = 0;

while(end_sim == 0)
    %---- Prepare sensor measurements
    attack                      = [0 ; attack_switch(end)*-100; 0];
    noise                       = noise_power * randn(p,1);
    
    y_SMT                       = sys_car.C * state_SMT   + attack + noise;
    y_noSMT                     = sys_car.C * state_noSMT + attack + noise;
    %---- Update the state of the three cars according to dynamics
    state                       = sys_car.A * state       + sys_car.B * Fx;
    state_SMT                   = sys_car.A * state_SMT   + sys_car.B * Fx_SMT;
    state_noSMT                 = sys_car.A * state_noSMT + sys_car.B * Fx_noSMT;
    
    %========== Leader Car ==========
    state(1) = mod(state(1),points(4));
    if(state(1) <= points(1))
        car.translation         = carCordInit + [0 0 state(1)];
        car.rotation            = [0, 1, 0, angles(1)];
        
        car2.translation        = car.translation;
        car2.rotation           = car.rotation;
        
        Fx                      = K_lqr  *([points(1); 5] - state);
        phase                   = 1;
    elseif(state(1) <= points(2))
        car.translation         = carCordInit + [0 0 points(1)] - [state(1) - points(1) 0 0];
        car.rotation            = [0, 1, 0, angles(2)];
        
        car2.translation        = car.translation;
        car2.rotation           = car.rotation;
        
        Fx                      = K_lqr  *([points(2); 5] - state);
        phase                   = 2;
        
    elseif(state(1) <= points(3))
        car.translation         = carCordInit + [points(1) - points(2) 0 points(1)] - [0 0 state(1) - points(2)];
        car.rotation            = [0, 1, 0, angles(3)];
        
        car2.translation        = car.translation;
        car2.rotation           = car.rotation;
        
        Fx                      = K_lqr  *([points(3); 5] - state);
        phase                   = 3;
    elseif(state(1) <= points(4))
        car.translation         = carCordInit + [points(1) - points(2) 0 points(1) - points(3) + points(2)] + [state(1) - points(3) 0 0];
        car.rotation            = [0, 1, 0, angles(4)];
        
        car2.translation        = car.translation;
        car2.rotation           = car.rotation;
        
        Fx                      = K_lqr  *([points(4); 5] - state);
        phase                   = 4;        
    end
    
    %========== Estimate the state of the blue car using SMT ==========
    [xhat_SMT, b]               = smt.addInputsOutputs(Fx_SMT, y_SMT);
    %-- Control the blue vehcile
    state_SMT(1)                = mod(state_SMT(1),points(4));
    xhat_SMT(1)                 = mod(xhat_SMT(1),points(4));
    if(state_SMT(1) <= points(1))
        car_SMT.translation     = carCordInit + [0 0 state_SMT(1)];
        car_SMT.rotation        = [0, 1, 0, angles(1)];
        
        error                   = state - [initShift;0] - xhat_SMT;
        Fx_SMT                  = K_lqr  *error;
        phase_SMT = 1;
    elseif(state_SMT(1) <= points(2))
        car_SMT.translation     = carCordInit + [0 0 points(1)] - [state_SMT(1) - points(1) 0 0];
        car_SMT.rotation        = [0, 1, 0, angles(2)];
        
        error                   = state - [initShift;0] - xhat_SMT;
        Fx_SMT                  = K_lqr  *error;
        
        phase_SMT = 2;
    elseif(state_SMT(1) <= points(3))
        car_SMT.translation     = carCordInit + [points(1) - points(2) 0 points(1)] - [0 0 state_SMT(1) - points(2)];
        car_SMT.rotation        = [0, 1, 0, angles(3)];
        
        error                   = state - [initShift;0] - xhat_SMT;
        Fx_SMT                  = K_lqr  *error;
        phase_SMT = 3;
    elseif(state_SMT(1) <= points(4))
        car_SMT.translation     = carCordInit + [points(1) - points(2) 0 points(1) - points(3) + points(2)] + [state_SMT(1) - points(3) 0 0];
        car_SMT.rotation        = [0, 1, 0, angles(4)];
        if(phase > 1)
            error               = state - [initShift;0] - xhat_SMT;
        else
            error               = [initShift - state(1);state(2)]  - [points(4) - xhat_SMT(1) ;xhat_SMT(2)];
        end
        Fx_SMT                  = K_lqr  *error;
        phase_SMT               = 4;
    end
    
    %========== Estimate the state of the red Car using Least squares ==========
    %-- Estimate using least squares
    Y_bar_noSMT                 = [Y_bar_noSMT(p+1:end); y_noSMT];
    xhat_noSMT                  = pinv(O_car)*Y_bar_noSMT;
    
    %---- Control the red vehicle
    state_noSMT(1)              = mod(state_noSMT(1),points(4));
    xhat_noSMT(1)               = mod(xhat_noSMT(1),points(4));
    
    if(state_noSMT(1) <= points(1))
        car_noSMT.translation   = carCordInit + [0 0 state_noSMT(1)];
        car_noSMT.rotation      = [0, 1, 0, angles(1)];
        
        error                   = state - [initShift;0] - xhat_noSMT;
        Fx_noSMT                = K_lqr  *error;
        phase_noSMT = 1;
    elseif(state_noSMT(1) <= points(2))
        if(time_counter < 17)
            time_counter        = time_counter + 1;
            attack_switch       = 1;
        else
            attack_switch       = 0;
        end
        
        car_noSMT.translation   = carCordInit + [0 0 points(1)] - [state_noSMT(1) - points(1) 0 0];
        car_noSMT.rotation      = [0, 1, 0, angles(2)];
        
        error                   = state - [initShift;0] - xhat_noSMT;
        Fx_noSMT                = K_lqr  *error;
        
        phase_noSMT             = 2;
    elseif(state_noSMT(1) <= points(3))
        attack_switch           = 1;
        
        car_noSMT.translation   = carCordInit + [points(1) - points(2) 0 points(1)] - [0 0 state_noSMT(1) - points(2)];
        car_noSMT.rotation      = [0, 1, 0, angles(3)];
        
        error                   = state - [initShift;0] - xhat_noSMT;
        Fx_noSMT                = K_lqr  *error;
        phase_noSMT = 3;
    elseif(state_noSMT(1) <= points(4))
        car_noSMT.translation   = carCordInit + [points(1) - points(2) 0 points(1) - points(3) + points(2)] + [state_noSMT(1) - points(3) 0 0];
        car_noSMT.rotation      = [0, 1, 0, angles(4)];
        
        if(phase > 1)
            error               = state - [initShift;0] - xhat_noSMT;
        else
            error               = [initShift - state(1);state(2)]  - [points(4) - xhat_noSMT(1) ;xhat_noSMT(2)];
        end
        Fx_noSMT                = K_lqr  *error;
        phase_noSMT             = 4;
    end
    
    %======== Detect crash ===========
    norm_error = norm(state(1) - state_noSMT(1));
    if(norm(state(1) - state_noSMT(1)) < 1)
        pause(3.0); % pause the scene for 3 seconds before re-initialize the simulation
        smt.flushBuffers();
        attack_switch           = 0;
        state                   = [initShift+5;   0];
        state_SMT               = [2;   0];
        state_noSMT             = [2;   0];
        time_counter            = 0;
    end
    
    %======== Update the Scene ==========
    camera.position             = car.translation + diffCamera;
    camera2.position            = car2.translation + diffCamera2;
    vrdrawnow;
    
    time                        = [time(2:end) time(end)+Ts];
    
    velocity_SMT_history        = [velocity_SMT_history(2:end)      state_SMT(2)];
    velocity_hat_SMT_history    = [velocity_hat_SMT_history(2:end)  xhat_SMT(2)];
    
    set(h1, 'YData',velocity_SMT_history);
    set(h11,'YData',velocity_hat_SMT_history);
    
    
    velocity_noSMT_history      = [velocity_noSMT_history(2:end)     state_noSMT(2)];
    velocity_hat_noSMT_history  = [velocity_hat_noSMT_history(2:end) xhat_noSMT(2)];
    
    set(h2, 'YData',velocity_noSMT_history);
    set(h22,'YData',velocity_hat_noSMT_history);
    
    
    attack_history              = [attack_history(2:end) attack(2)];
    set(h3,'YData',attack_history);
    drawnow; 
    
    
    pause(0.1);
end
