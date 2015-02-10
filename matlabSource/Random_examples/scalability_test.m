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


%% Housekeeping
clc; close all; clear all;
rand('state',0);
randn('state',0);
imhotepSMTPath = './';  %path to Imhotep-SMT
addpath(imhotepSMTPath);	


%% In this test, the number of sensors is fixed to 20 and we increase the 
% number of states from 10 - 150. The systems are pre-simulated and the
% inputs and outputs are recorded.

% Generate the system
format longg
test_counter        = 0;
TimeSpent_SMT_test1 = [];
p = 20;     % the number of sensors is fixed to 20, 9 of them under attack
for n = [10, 25, 50, 75, 100, 150]
    % load the system
    load(['./Test1_states/test_n' num2str(n) '_p' num2str(p)]); 
    test_counter = test_counter + 1;
    
    % configure ImhotepSMT
    noiseBound                  = zeros(p,1);       % noise bounds
    max_sensors_under_attack    = int8(8);          % maximum sensors under attack

    safe_sensors                = [];

    smt = ImhotepSMT();
    smt.init(sys, max_sensors_under_attack, safe_sensors, noiseBound )

    for t = 1 : n
        % load the outputs from collected data
        y = Y(:,t);
        tic;
        [xhat, b] = smt.addInputsOutputs(0, y);
        TimeSpent_SMT_test1(test_counter) = toc;
    end
    
end 

% Plot figure
figure;
plot([10, 25, 50, 75, 100, 150], TimeSpent_SMT_test1,'LineWidth',3)
set(gca,'FontSize',30);
xlabel('number of states');
ylabel('Time (sec)');


%% In this test, the number of states is fixed to 50 and we increase the 
% number of sensors from 3 - 150. The systems are pre-simulated and the
% inputs and outputs are recorded.

% Generate the system
test_counter        = 0;
TimeSpent_SMT_test2 = [];
n = 20;     % the number of sensors is fixed to 20, 9 of them under attack
for p = [3, 30, 60, 90, 120, 150]
    % load the system
    load(['./Test2_sensors/test_n' num2str(n) '_p' num2str(p)]); 
    test_counter = test_counter + 1;
    
    % configure ImhotepSMT
    noiseBound                  = zeros(p,1);       % noise bounds
    max_sensors_under_attack    = int8(s);          % maximum sensors under attack

    safe_sensors                = [];

    smt = ImhotepSMT();
    smt.init(sys, max_sensors_under_attack, safe_sensors, noiseBound )

    for t = 1 : n
        % load the outputs from collected data
        y = Y(:,t);
        tic;
        [xhat, b] = smt.addInputsOutputs(0, y);
        TimeSpent_SMT_test2(test_counter) = toc;
    end
    
end 


% Plot figure
figure;
plot([3, 30, 60, 90, 120, 150], TimeSpent_SMT_test2,'LineWidth',3)
set(gca,'FontSize',30);
xlabel('number of sensors');
ylabel('Time (sec)');

