clc
close all
clear all

rand('state',0);
randn('state',0);
%% PARAMETERS

% Motor Parameters
Lm          = 1E-3;         % Motor Inducatnce
Rm            = 3.1;          % Motor resistance
Jg          = 1.2E-6;       % Angular mass of gears
Bg          = 6.7E-7;       % Mechanical resistance of gears to rotate
g           = 204;          % gear ratio
alpha       = 1E-3;         % Current-Torque ratio of the Motor
radius      = 0.015;        % radius of sprocket wheel
Kt          = 1E-3;         % Compliance of the track

% Chasis Parameters
Mt          = 0.8;          % Mass of the tank
Jt          = 5E-4;         % Angular mass of the Tank
Width       = 0.1;          % Width of the tank (B)
Br          = 1;            % Mechanical friction of the tracks to roll
Bs          = 14;           % Mechanical friction of the tracks to slide
Bl          = 0.7;          % Mechanical friction of the tracks to turn
Sl          = 0.3;          % Lateral friction of Tracks

%% Mechanical Model SYSTEM Kineamtics + Dynamics
A_mech      = [0        1; 0    -Br/Mt];
B_mech      = [0         ; 1/Mt      ];
C_mech      = [1        0; 0        1;  0       1];   %GPS for x, Encoder for v
D_mech      = [0         ; 0         ;  0       ];

A_rotation  = [0        1; 0    -Bl/Jt];
B_rotation  = [0         ; 1/Jt      ];
C_rotation  = [1        0; 0        1];
D_rotation  = [0         ;          0];


%% Dynamics with two Motors + Mechanical (Electrical + Mechanical Dynamics) + Kinematics

A_1 = [0 1 0 0 0 0 0 0];
A_2 = [0 -Br/Mt 0 0 1/Mt 0 0 1/Mt];

A_3 = [0 0 -Rm/Lm -alpha/Lm 0 0 0 0];
A_4 = [0 0 alpha/Jg -Bg/Jg -radius/(g*Jg) 0 0 0];
A_5 = [0 0 0 radius/(g*Kt) 0 0 0 0];

A_6 = [0 0 0 0 0 -Rm/Lm -alpha/Lm 0];
A_7 = [0 0 0 0 0 alpha/Jg -Bg/Jg -radius/(g*Jg)];
A_8 = [0 0 0 0 0 0 radius/(g*Kt) 0];

A_full = [A_1; A_2; A_3; A_4; A_5; A_6; A_7; A_8];
B_full = [0; 0; 1/Lm ; 0; 0; 1/Lm; 0; 0];

C_full = [1 0 0 0 0 0 0 0;...
          0 0 0 1 0 0 0 0;...
          0 0 0 0 0 0 1 0;...
          0 0 1 0 0 0 0 0;...
          0 0 0 0 0 1 0 0];
D_full = 0;%zeros(length(A_full),1);

Ts          = 0.01;

system_type = 1;

if(system_type == 1)
    A = A_mech; B = B_mech; C = C_mech; D = D_mech;
elseif(system_type == 2)
    A = A_rotation; B = B_rotation; C = C_rotation; D = D_rotation;
else
    A = A_full; B = B_full; C = C_full; D = D_full;
end

% Make sure A has spectral radius 1 (otherwise A^k will be either very
% large or very small for k large)
% A           = A/max(abs(eig(A)));

n           = length(A); % Size of system
[p p_c]     = size(C);


Sys  = ss(A,B,C,D);
SysD = c2d(Sys, Ts, 'ZOH');
Ad = SysD.A;    Bd = SysD.B;    Cd = SysD.C;    Dd = SysD.D;

Sys_rotation = ss(A_rotation, B_rotation, C_rotation, D_rotation);
Sys_rotation_disc = c2d(Sys_rotation, Ts, 'ZOH');

x0 = zeros(n,1);

Cd_sensor2 = Cd([1,3],:);
Cd_sensor3 = Cd([1,2],:);

SysD_sensor2 = ss(Ad, Bd, Cd_sensor2, 0, Ts);
SysD_sensor3 = ss(Ad, Bd, Cd_sensor3, 0, Ts);


%% Check for some conditions:
% check number of correctable errors q


%% Construct the Annihilator matrix (PHI) and the Input weight matrix
Obsv = [];      IN = [];
Ablkdiag = [];
Bblkdiag = [];
Cblkdiag = [];
for t = 1 : n
    Obsv = [Obsv; Cd*(Ad^(t-1))];
    
    in_row = zeros(p, t);
    in_row_index = 1;
    for(count_in = t-2 : -1 : 0)
        in_row(:,in_row_index) = Cd*(Ad^count_in)*Bd;
        in_row_index = in_row_index+1;
    end
    if(length(IN) == 0)
        IN = in_row;
    else
        IN = [IN zeros((t-1)*p,1); in_row];
    end
    
    
    Atilde = [eye(n,n) zeros(n,n*(t-1)); Ablkdiag zeros(n*(t-1),n)];
    Btilde = [zeros(n,1); Bblkdiag];
    Cblkdiag = blkdiag(Cblkdiag, Cd);
    Ablkdiag = blkdiag(Ablkdiag, Ad);
    Bblkdiag = [Bblkdiag ; Bd];        
end

% We create the Annihilator of the Observability Matrix
MO = eye(length(Obsv)) - Obsv*inv(Obsv'*Obsv)*Obsv';

% We need the Annihilator to have more columns than rows, use SVD to get rid
% of rows which has zero Singular Value
S_svd = svd(MO);
index = find (S_svd > 1E-6);
[U_svd,S_svd,V_svd] = svd(MO);
MO_reduced = U_svd(index,:)*S_svd*V_svd';
PHI = MO_reduced;

O_INV =  pinv(Obsv);

save 'demo_param'
% save 'counter_example'

%% Design Controller
Q = eye(n);
R = eye(1);
[K_linear,S,e] = dlqr(Ad,Bd,Q,R,zeros(1)); % gain normal controller for in order

cl_sys_linear = ss(Ad-Bd*K_linear, [1;0], Cd, Dd,Ts);
% step(cl_sys_linear)

[K_rotation,S,e] = dlqr(Sys_rotation_disc.a,Sys_rotation_disc.b,Q,R,zeros(1)); % gain normal controller for in order
cl_sys_rotation = ss(Sys_rotation_disc.a-Sys_rotation_disc.b*K_rotation, [1;0], Sys_rotation_disc.c, Sys_rotation_disc.d,Ts);
% step(cl_sys_rotation)

sys_rotation    = Sys_rotation_disc;
sys_linear      = SysD;

sys = sys_linear;

preconditioning         = 0;

%% augmented system
shift_matrix            = [zeros(n*p-p,p) eye(n*p-p); zeros(p,n*p)];
% Cd = rand(3,2);

% W                        = [1 0 0 ; 0 1 0; 0 0 5];
% Cd                      = W*Cd;
% A_bar                   = [sys.A, zeros(n,n*p); [zeros(n*p-p,n); -1*sys.C*(sys.A^(n))], shift_matrix];
A_bar                   = [Ad, zeros(n,n*p); [zeros(n*p-p,n); -1*Cd*(Ad)^2], shift_matrix];
% H                     = [obsv(sys.A,sys.C)*(sys.A)^(n+1), eye(n*p)];
% Cd = [ 1 0; 1 0; 1 0; 0 1; 0 1];
% p = 5;

Cd = [ 1 0; 0 1; 0 1];
p = 3;


C_bar                   = [obsv(Ad,Cd), eye(n*p)];
% C_bar                   = [obsv(A,C)*inv(A^(n-1)), eye(n*p)];
% C_bar                   = [obsv(A,C), eye(n*p)];
if (preconditioning == 1)
    Q                       = C_bar;
    M                       = blkdiag(eye(n), 10*eye(p*n));
    C_bar                   = M * (C_bar' * C_bar);
end
B_bar                   = [[Bd; zeros((n-1)*p,1);-Cd*Ad*Bd], [zeros(n+n*p-p,p);eye(p)]];
B_bar                   = [[zeros(n,1); zeros((n-1)*p,1);-Cd*Bd] B_bar];
% O_bar                   = obsv(A_bar, C_bar);
sys_bar                 = ss(A_bar, B_bar, C_bar, 0, Ts);

%%

max_qs                  = 1;
% sys = ss(A,zeros(n,1),C,0, 0.1);
% check = [];
% min_eval = [];
% min_eval_P = [];
% [symbols, permutations, number_of_permutations] = generateExhaustivePermutation (1:p, 2*max_qs);
% permutations = permutations(:,1:2*max_qs);
%  
% T = n;
% P = C_bar'*C_bar;
% 
% for k = 1 : length(permutations)
%     per = permutations(k,:);
%     all_indecies = 1:n*p;
%     indecies_to_be_kept = reshape(kron(([1:T]-1)*p, ones(2*max_qs,1))+per'*ones(1,T),1,2*max_qs*n);
%     indecies_to_be_removed = n + setdiff(all_indecies,indecies_to_be_kept);
% 
% 
%     P_rest = P;
%     P_rest(indecies_to_be_removed,:) = [];
%     P_rest(:,indecies_to_be_removed) = [];
%     
% %     P = [O'*O, O_gamma'; O_gamma, eye(r)];
%     
%     min_eval_P(k) = min(eig(P_rest));
%     max_eval_P(k) = max(eig(P_rest));
% end
% index = find(min_eval_P > 1E-3);
% delta = min(min_eval_P(index))
% delta2 = max(eig(C_bar'*C_bar))
% sigma = max(svd(A_bar))
% alpha = 0.5*(1/delta2 + 1/delta2*(1 - 1/sigma));
% 
% raho = (2*sigma + 1)/(2*sigma - 1)
% 
% ratio = delta2/delta
% 
% 
% pp = size(C_bar',2); 
% alpha_I = alpha*eye(pp);
        

%%
tau = n;
O_SMT = [];
for t = 1 : 1 : tau
    O_SMT = [O_SMT Cd*Ad^(t-1)];
end

% construct the data structures
OO = O_SMT;
O_SMT = [];
for counter = 1 : p
    O_SMT{counter}    = [reshape(OO(counter,:), n,tau)'];
end

%%

% %% Construct the Annihilator matrix (PHI) and the Input weight matrix
% Obsv = [];      IN = [];
% Ablkdiag = [];
% Bblkdiag = [];
% Cblkdiag = [];
% p = 5;
% n = 8;
% for t = 1 : n
%     Obsv = [Obsv; Cd*(Ad^(t-1))];
%     
%     in_row = zeros(p, t);
%     in_row_index = 1;
%     for(count_in = t-2 : -1 : 0)
%         in_row(:,in_row_index) = Cd*(Ad^count_in)*Bd;
%         in_row_index = in_row_index+1;
%     end
%     if(length(IN) == 0)
%         IN = in_row;
%     else
%         IN = [IN zeros((t-1)*p,1); in_row];
%     end
%     
%     
%     Atilde = [eye(n,n) zeros(n,n*(t-1)); Ablkdiag zeros(n*(t-1),n)];
%     Btilde = [zeros(n,1); Bblkdiag];
%     Cblkdiag = blkdiag(Cblkdiag, Cd);
%     Ablkdiag = blkdiag(Ablkdiag, Ad);
%     Bblkdiag = [Bblkdiag ; Bd];        
% end
% 
% % We create the Annihilator of the Observability Matrix
% MO = eye(length(Obsv)) - Obsv*inv(Obsv'*Obsv)*Obsv';
% 
% % We need the Annihilator to have more columns than rows, use SVD to get rid
% % of rows which has zero Singular Value
% S_svd = svd(MO);
% index = find (S_svd > 1E-6);
% [U_svd,S_svd,V_svd] = svd(MO);
% MO_reduced = U_svd(index,:)*S_svd*V_svd';
% PHI = MO_reduced;
% 
% O_INV =  pinv(Obsv);

%% Check for some conditions:
% check number of correctable errors q
% 
% [V, lamda] = eig(Ad);
% v1 = V(:,1);
% v2 = V(:,2);
% v3 = V(:,3);
% v4 = V(:,4);
% v5 = V(:,5);
% v6 = V(:,6);
% v7 = V(:,7);
% v8 = V(:,8);
% 
% support1 = length(find(Cd*v1 ~= 0));
% support2 = length(find(Cd*v2 ~= 0));
% support3 = length(find(Cd*v3 ~= 0));
% support4 = length(find(Cd*v4 ~= 0));
% support5 = length(find(Cd*v5 ~= 0));
% support6 = length(find(Cd*v6 ~= 0));
% support7 = length(find(Cd*v7 ~= 0));
% support8 = length(find(Cd*v8 ~= 0));
% 
% 
% support = [support1 support2 support3 support4 support5 support6 support7 support8]
% 
% Cd_sensor2 = Cd([1,3,4,5],:);
% Cd_sensor3 = Cd([1,2,4,5],:);
% Cd_sensor_both = Cd([1,4,5],:);
% 
% SysD_sensor2 = ss(Ad, Bd, Cd_sensor2, 0, Ts);
% SysD_sensor3 = ss(Ad, Bd, Cd_sensor3, 0, Ts);
% SysD_sensor_both = ss(Ad, Bd, Cd_sensor_both, 0, Ts);
% 
% O2 = obsv(SysD_sensor2);
% O2_INV = pinv(O2);
% r2 = rank(O2);
% 
% O3 = obsv(SysD_sensor3);
% O3_INV = pinv(O3);
% r3 = rank(O3);
% 
% O_both = obsv(SysD_sensor_both);
% O_both_INV = pinv(O_both);
% r_both = rank(O_both);
% 
% obsv_rank = [r2 r3 r_both];

