%% Housekeeping
clc; close all; clear all;
rand('state',0);
randn('state',0);
addpath('../../');

%% load the system
load Atilde.mat
load Ctilde.mat

Ts          = 0.5;
A           = expm(Atilde*Ts);
B           = zeros(size(A,2),1);
C           = Ctilde;

n           = size(A,1);
p           = size(C,1);
m           = size(B,2);

sys         = ss(A,B,C,0, Ts);
x0          = randn(n,1);


s           = int8(14);
noiseBound  = zeros(p,1);

%%
per         = randperm(size(C,1)-1);
K           = per(1:s);
attackpower = 50;

%%
smt = ImhotepSMT();
smt.init(sys, s, [35], noiseBound )

%%
x   = x0;
X   = []; Y = []; U = [];
Xhat = [];
Bhat = [];
error = [];
errorLS = [];


inputpower = 1;
u   = zeros(m,1);
y   = zeros(p,1);

Nsim    = 200;

O           = obsv(A,C);
Y_ls        = zeros(p*n,1);

for t = 1 : Nsim
    X   = [X x];
    U   = [U u];
    % Generate a random attack vector supported on K
    a = zeros(p,1);
    a(K) = attackpower*randn(length(K),1);
    % The measurement is y=C*x+a
    y   = C*x + a;
    Y   =   [Y y];
    
    Y_ls = [Y_ls(p+1:end); y];
   
    
    u   = randn(m,1)*inputpower;
    x   = A*x + B*u;
    
    [xhat, bhat] = smt.addInputsOutputs(u, y);
    error(t) = norm(xhat - x);
    
    Xhat =[Xhat xhat];
    Bhat{t} = bhat;
    
    xhatLS = O\Y_ls;
    errorLS(t) = norm(xhatLS - x);
end

time = 0 : Ts : (Nsim-1)*Ts;
errorLS = [time', errorLS'];
error = [time', error'];


save('error_imhotepSMT.txt', 'error','-ascii','-double')
save('error_LS.txt', 'errorLS','-ascii','-double')
