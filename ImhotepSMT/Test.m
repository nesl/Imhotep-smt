%% Housekeeping
clc; close all; clear all;
rand('state',0);
randn('state',0);


%% Generate the system
n           = 2;   % Size of system
p           = 5;   % Number of sensors
m           = 3;    % number of inputs

% Generate a system with a random A matrix
A           = full(sprand(n,n,0.3)); %randn(n,n);
% Make sure A has spectral radius 1 (otherwise A^k will be either very
% large or very small for k large)
A           = A/(max(abs(eig(A))) + 0.1);
% The 'C' matrix of the system
%C           = full(sprand(p,n,0.5)); %randn(p,n);
C           = randn(p,n);
B           = randn(n,m);

sys         = ss(A,B,C,0, 0.1);
x0          = randn(n,1);

s           = int8(p/3);
noiseBound  = zeros(p,1);

%%
per = randperm(p);
K = per(1:s);
attackpower = 50;
%%
smt = ImhotepSMT();
smt.init(sys, noiseBound, [], s)

%%
x   = x0;
X   = []; Y = []; U = [];
Xhat = [];
Bhat = [];

inputpower = 1;
u   = zeros(m,1);
y   = zeros(p,1);

for t = 1 : 100*n
    X   = [X x];
    U   = [U u];
    % Generate a random attack vector supported on K
    a = zeros(p,1);
    a(K) = attackpower*randn(length(K),1);
    % The measurement is y=C*x+a
    y   = C*x + a;
    Y   =   [Y y];
    
    
   
    
    u   = randn(m,1)*inputpower;
    x   = A*x + B*u;
    
    [xhat, bhat] = smt.addInputsOutputs(u, y);
    error = norm(xhat - x)
    Xhat =[Xhat xhat];
    Bhat{t} = bhat;
end

