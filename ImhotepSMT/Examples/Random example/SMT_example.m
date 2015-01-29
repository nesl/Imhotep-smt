%% Housekeeping
clc; close all; clear all;
rand('state',0);
randn('state',0);


%% Generate the system
n           = 20;   % Size of system
p           = 25;   % Number of sensors
tau         = n;
% Generate a system with a random A matrix
A           = full(sprand(n,n,0.3)); %randn(n,n);
% Make sure A has spectral radius 1 (otherwise A^k will be either very
% large or very small for k large)
A           = A/(max(abs(eig(A))) + 0.1);
% The 'C' matrix of the system
C           = randn(p,n);
%C           = full(sprand(p,n,0.5)); %randn(p,n);
O           = obsv(A,C);
I           = eye(p*tau);
Q           = [O I];

sys         = ss(A,zeros(n,1),C,0, 0.1);
x0          = randn(n,1);

attackpower = 20;   % Magnitude of the attacks (i.e., norm of the attack vector)
max_s       = floor(p/2-1);    % Number of attacked sensors

%% Simulate the System under attack
    
    
for s = 1 : max_s
    % Choose a random attacking set K of size qs
    per = randperm(p);
    K = per(1:s);
    %K  = attackSupport{s}
    % Choose an initial condition
    x = x0;
    X = []; Y = []; E = []; O = [];
    for t = 1 : 1 : tau
        % Generate a random attack vector supported on K
        a = zeros(p,1);
        a(K) = attackpower*randn(length(K),1);
        % The measurement is y=C*x+a
        y = C*x + a;
        % Update the arrays X,Y,E
        X = [X; x];
        Y = [Y y];
        E = [E a];
        O = [O C*A^(t-1)];
        x = A*x;
    end
    
    % construct the data structures
    YY = Y; OO = O;
    Y = []; O = [];
    for counter = 1 : p
        Y{counter}    = YY(counter,:)';
        O{counter}    = [reshape(OO(counter,:), n,tau)'];
    end
    
    %profile -memory on
    % Initialze the SMT solver
    smt = SMT();
    smt.init(n,p,tau,max_s);
    for counter = 1 : p
        smt.addSensorMeasurements(Y{counter}, O{counter}, 0, counter);
    end

    tstart = tic;
    
    [xhat, Khat] = smt.solve();
    
    %profreport
    
    telapsed(s) = toc(tstart)
    
    KK{s}  = K;
    KKhat{s} = Khat;
    error{s} = norm(x0 - xhat);
end


%% Plot figure

cfigure(50,70);
stem(1:max_s, telapsed,'LineWidth',3)
set(gca,'FontSize',30);
xlabel('# sensors under attack');
ylabel('Time (sec)');
%saveplot('SMT_executionTime')



