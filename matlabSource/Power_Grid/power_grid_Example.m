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

%   This is an example of using Imhotep-SMT in estimating the state of the
%   IEEE 14-bus power network. The bus composed of 5 synchronous generators 
%   and 14 buses. The state of each generator includes rotor angle and 
%   frequency.  The overall system has 35 sensors: 14 sensors measure the 
%   real power injections at every bus, 20 sensors measure the real power 
%   flows along every branch, and one sensor measures the rotor angle of 
%   generator number 1. The matrices A, B, and C  modeling the power 
%   network are derived in:
%       F. Pasqualetti, F. Dorfler, and F. Bullo. "Attack detection and 
%       identification in cyber- physical systems". IEEE Transactions on 
%       Automatic Control, 58(11):2715-2729, Nov 2013.
%   where it is also shown that the rotor angle sensor must be secured for 
%   the system to have non-zero security index. In this simulation, the 
%   attacker is assumed to pick a random set of 14 sensors.  

%   Author: Yasser Shoukry (yshoukry@ucla.edu)
%   Date: Feburary 5, 2015

%% Housekeeping
clc; close all; clear all;
rand('state',0);
randn('state',0);
imhotepSMTPath = './';  %path to Imhotep-SMT
addpath(imhotepSMTPath);	

%% load the description of the IEEE 14 bus system
load Atilde.mat
load Ctilde.mat

Ts                          = 0.5;
A                           = expm(Atilde*Ts);
B                           = zeros(size(A,2),1);
C                           = Ctilde;


n                           = 10;    % number of states
p                           = 35;    % number of sensors



sys                         = ss(A,B,C,0, Ts);  % create the sysetm
noiseBound                  = zeros(p,1);       % noise bounds
max_sensors_under_attack    = int8(14);         % maximum sensors under attack

safe_sensors                = [35];
%% Initialize Imhotep-SMT solver
smt = ImhotepSMT();
smt.init(sys, max_sensors_under_attack, safe_sensors, noiseBound )

%% Attacker
per                         = randperm(p-1); % the attacker randmoizes the sensor indecies
attacked_sensor_index       = per(1:max_sensors_under_attack); % the attacker then picks the first 14 sensors to attack them
attackpower                 = 50;

%% Simulate the IEEE bus network
x                           = randn(n,1); % unknown initial condition
simulation_time             = 200; %simulation time (number of samples)


state_error_ImhotepSMT      = [];
state_error_LeastSquares    = [];
O                           = obsv(A,C);
Y_ls                        = zeros(p*n,1);


for t = 1 : simulation_time
    % Generate a random attack vector
    attack_signal                           = zeros(p,1);
    attack_signal(attacked_sensor_index)    = attackpower*randn(length(attacked_sensor_index),1);
    
    % the attacker corrupts the measurements
    y                                       = C*x + attack_signal;

    % the system has no inputs. Set the input signal u to zero
    u                                       = zeros(size(B,2),1);
    
    % simulate the system
    x                                       = A*x + B*u;
    
    % estimate the state using Imhotep-SMT
    [xhat, bhat]                            = smt.addInputsOutputs(u, y);
    state_error_ImhotepSMT(t)               = norm(xhat - x);
    
    % estimate the state using Least squares
    Y_ls                                    = [Y_ls(p+1:end); y];
    xhatLS                                  = O\Y_ls;
    state_error_LeastSquares(t)             = norm(xhatLS - x);
end


%% Plot the results
time = 0 : Ts : (simulation_time-1)*Ts;

plot(time, state_error_ImhotepSMT, 'LineWidth',3);
hold on
plot(time, state_error_LeastSquares, ':r', 'LineWidth',3);
title('Estimation error Imhotep-SMT vs Least squares')
ylabel('Estimation error')
xlabel('time (s)')
legend('Imhotep-SMT', 'least squares')
