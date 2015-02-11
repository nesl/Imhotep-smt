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


%% Step 1: Initialize the Solver
smt         = ImhotepSMT();

%% Step 2: Offline Configuration of the Solver
A           = [1 0.0099; 0 0.9876];
B           = [0.0001; 0.0124];
C           = [1 0; 0 1; 0 1];
ugv         = ss(A,B,C,0,0.001);


max_sensors_under_attack = int8(1); % maximum sensors under attack
safe_sensors             = [];
noise_bound              = [0; 0; 0];      

smt.init(ugv, max_sensors_under_attack, safe_sensors, noise_bound);

%% Step2b: Specify safe sensors
safe_sensors             = [1];
smt.init(ugv, max_sensors_under_attack, safe_sensors, noise_bound);

smt.checkObservabilityCondition();

%% Step3: Simulate the system
error       = [];
x0          = randn(2,1); %unknown initial condition
x           = x0;
attacked_sensor_index = 2; % the attacker chooses to attack the second sensor.
simulation_time         = 50;
for t = 1 : simulation_time	% simulate the system for 1000 time steps
    y       = C*x;		% the system measurements 
    % the attacker corrupts the second sensor with random data
    y(attacked_sensor_index) = y(attacked_sensor_index) + 50*randn(1); 
    
    F       = 1;		% the force supplied to the UGV
    x       = A*x + B*F;	% simulate the system
    
    [xhat, sensor_under_attack] = smt.addInputsOutputs(F, y);
    error(t) = norm(xhat - x);
end


%% Plot the results
time = 0 : 0.001 : (simulation_time-1)*0.001;

plot(time, error, 'LineWidth',3);
title('Estimation error Imhotep-SMT')
ylabel('Estimation error')
xlabel('time (s)')
