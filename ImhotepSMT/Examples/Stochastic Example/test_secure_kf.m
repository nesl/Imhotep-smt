clc; close all; clear all;
rand('state',0);
randn('state',0);

%load './Test2_sensors/test_n20_p3.mat'

n = 20;
p = 5;


attackpower = 20;   % Magnitude of the attacks (i.e., norm of the attack vector)
s           = 2;


A           = full(sprand(n,n,0.3)); %randn(n,n);
% Make sure A has spectral radius 1 (otherwise A^k will be either very
% large or very small for k large)
A           = A/(max(abs(eig(A))) + 0.1);
% The 'C' matrix of the system
C           = full(sprand(p,n,0.2)); %randn(p,n);
Ts          = 0.1;

sys         = ss(A,zeros(n,1),C,0, Ts);
x0          = randn(n,1);

CC = {};

combinations = combnk(1:p,p-s);
indecies = cell([size(combinations,1) 1]);

for counter = 1 : size(combinations,1)
    indecies{counter} = combinations(counter,:);
end
% indecies{1} = [3,4,5];
% indecies{2} = [2,4,5];
% indecies{3} = [2,3,5];
% indecies{4} = [2,3,4];
% indecies{5} = [1,4,5];
% indecies{6} = [1,3,5];
% indecies{7} = [1,3,4];
% indecies{8} = [1,2,5];
% indecies{9} = [1,2,4];
% indecies{10} = [1,2,3];

%systems
for counter = 1 : length(indecies)
    CC{counter} = C(indecies{counter},:);
end

%
Q = eye(n);
R = eye(p-s);


Plant   = {};
kalmf   = {};
L       = {};
P       = {};

for counter = 1 : length(CC)
    Plant{counter} = ss(A,[zeros(n) eye(n)],CC{counter},0,-1);
    [kalmf{counter},L{counter},P{counter}] = kalman(Plant{counter},Q,R);  
end

% Choose a random attacking set K of size qs
per         = randperm(p);
K           = per(1:s);

% Choose an initial condition
x           = x0;
Y           = [];

tau         = 100;

for counter = 1 : length(CC)
    x_hat{counter}    = zeros(n,1);
end

for t = 1 : 1 : tau
    t
    % Generate a random attack vector supported on K
    a       = zeros(p,1);
    a(K)    = attackpower*randn(length(K),1);
    % The measurement is y=C*x+a
    y       = C*x + a + randn*eye(p,1);
    % Update the arrays X,Y,E
    Y       = [Y y];

    for counter = 1 : length(CC)
        x_hat{counter}      = A*x_hat{counter} + L{counter}*(y(indecies{counter}) - CC{counter}*x_hat{counter});
        X_hat{counter}(t,:) = x_hat{counter}';
    end
    x       = A*x + randn*eye(n,1);
   
    X_hist(t,:) = x';
end
%%

tstart = tic;

M = {};
cov_out_star= {};
cov_out = {};
cov = {};
error = [];

for counter = 1 : length(CC)
    cov{counter}{1} = zeros(n); 
    error(counter,1)    = max(max(abs(cov{counter}{1} - P{counter})));
end
    

for time = 1 : tau
    for counter = 1 : length(CC)
        cov{counter}{time+1}        = cov{counter}{time} + X_hat{counter}(time,:) * X_hat{counter}(time,:)';
        error(counter,time + 1)    = max(max(max(abs(1/time * cov{counter}{time+1} - P{counter}))) - 1.3, 0);
    end
end

tend = toc(tstart)

figure; hold on
plot([0:tau], error(1,:)', [0:tau], error(2,:)', [0:tau], error(3,:)', [0:tau], error(4,:)', [0:tau], error(5,:)', [0:tau], error(6,:)', [0:tau], error(7,:)',[0:tau], error(8,:)',[0:tau], error(9,:)',[0:tau], error(10,:)')
legend('1','2','3','4','5','6','7','8','9','10');


figure; hold on
plot([0:tau-1], X_hat{1}(:,1), [0:tau-1], X_hat{2}(:,1)', [0:tau-1], X_hat{3}(:,1)', [0:tau-1], X_hat{1}(:,4)', [0:tau-1], X_hat{1}(:,5)', [0:tau-1], X_hat{6}(:,1)', [0:tau-1], X_hat{7}(:,1)',[0:tau-1], X_hat{8}(:,1)',[0:tau-1], X_hat{9}(:,1)',[0:tau-1], X_hat{10}(:,1)')
legend('1','2','3','4','5','6','7','8','9','10');

plot([0:tau-1], X_hist(:,1), ':')


%% safe set = 2
state = [3, 4, 8];
for counter = state
    figure; hold on
    plot([0:tau-1], X_hat{2}(:,counter)')
    plot([0:tau-1], X_hist(:,counter), ':')
    
    data = [Ts*[0:tau-1]', X_hat{2}(:,counter)];
    save(['xhat_' num2str(counter)  '.txt'], 'data','-ascii','-double')
    
    data = [Ts*[0:tau-1]', X_hist(:,counter)];
    save(['x_' num2str(counter)  '.txt'], 'data','-ascii','-double')

end

    
%%
state = 4;

figure; hold on
plot([0:tau-1], X_hat{2}(:,state)')
plot([0:tau-1], X_hist(:,state), ':')

%%
state = 8;
figure; hold on
plot([0:tau-1], X_hat{2}(:,state)')
plot([0:tau-1], X_hist(:,state), ':')


% for counter = 1 : length(CC)
% 
% 
% 
% cov{counter}        = 1/tau * cov{counter};
% M{counter}          = eye(n* (p - 2));
% cov_out{counter}    = obsv(A,CC{counter}) * cov{counter} * obsv(A,CC{counter})' + M{counter};
% cov_out_star{counter} = obsv(A,CC{counter}) * P{counter} * obsv(A,CC{counter})' + M{counter};
% end

return

for counter = 1 : length(CC)
    data = [Ts*[0:tau-1]', error(counter,[1:tau])'];
    save(['error_' num2str(counter) '.txt'], 'data','-ascii','-double')
end

xx = [ 
    41.9454
    8.3405
    7.5900
    7.5843
    7.7602
];

mean(xx)

max(xx - mean(xx))
