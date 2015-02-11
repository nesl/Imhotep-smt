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

%   This file generates multiple test cases in order to test the
%   scalability of Imhotep-SMT


%% Generate the system for Test Case 1
clc; close all; clear all;
rand('state',0);
randn('state',0);

for n = [10, 25, 50, 75, 100, 150]
    clear X  Y  E O Y_ETPG Ablkdiag Cblkdiag
    p           = 20;
    tau         = n;
    % Generate a system with a random A matrix
    A           = full(sprand(n,n,0.3)); %randn(n,n);
    % Make sure A has spectral radius 1 (otherwise A^k will be either very
    % large or very small for k large)
    A           = A/(max(abs(eig(A))) + 0.1);
    % The 'C' matrix of the system
    C           = full(sprand(p,n,0.2)); %randn(p,n);
    Ts          = 0.1;

    sys         = ss(A,zeros(n,1),C,0, Ts);
    x0          = randn(n,1);

    attackpower = 20;   % Magnitude of the attacks (i.e., norm of the attack vector)
    max_s       = floor(p/2-1)-1;
    s           = max_s;

    % Choose a random attacking set K of size qs
    per         = randperm(p);
    K           = per(1:s);

    % Choose an initial condition
    x = x0;
    Y = [];
    
    for t = 1 : 1 : tau
        t
        % Generate a random attack vector supported on K
        a       = zeros(p,1);
        a(K)    = attackpower*randn(length(K),1);
        % The measurement is y=C*x+a
        y       = C*x + a;
        % Update the arrays X,Y,E
        Y       = [Y y];
        
        x       = A*x;
    end
    
    save(['./Test1_states/test_n' num2str(n) '_p' num2str(p)]);
end


%% Generate the system for Test Case 2
clc; close all; clear all;
rand('state',0);
randn('state',0);

for p = [3, 30, 60, 90, 120, 150]
    n           = 20;
    tau         = n;
    % Generate a system with a random A matrix
    A           = full(sprand(n,n,0.3)); %randn(n,n);
    % Make sure A has spectral radius 1 (otherwise A^k will be either very
    % large or very small for k large)
    A           = A/(max(abs(eig(A))) + 0.1);
    % The 'C' matrix of the system
    C           = full(sprand(p,n,0.5)); %randn(p,n);

    sys         = ss(A,zeros(n,1),C,0, 0.1);
    x0          = randn(n,1);

    attackpower = 20;   % Magnitude of the attacks (i.e., norm of the attack vector)
    max_s       = floor(p/3-1);
    s           = max_s;
    % Simulate the System under attack

    % Choose a random attacking set K of size qs
    per         = randperm(p);
    K           = per(1:s);
    
    % Choose an initial condition
    x           = x0;
    Y           = [];

    for t = 1 : 1 : tau
        t
        % Generate a random attack vector supported on K
        a       = zeros(p,1);
        a(K)    = attackpower*randn(length(K),1);
        
        % The measurement is y=C*x+a
        y = C*x + a;
        % Update the arrays X,Y,E
        Y = [Y y];
        
        x = A*x;
    end
    save(['./Test2_sensors/test_n' num2str(n) '_p' num2str(p)]);
end


