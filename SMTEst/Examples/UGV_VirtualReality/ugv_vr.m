%% Book  keeping
clc; close all; clear all; rand('state',0); randn('state',0);

%% Initialize 3D model
world = vrworld('ugv_vr2.wrl');
open(world);

fig = view(world, '-internal');
vrdrawnow;

car = vrnode(world, 'Automobile2');
car_noSMT = vrnode(world, 'Automobile');
camera = vrnode(world, 'View1');


cameraCordInit = [2.4, 10.2, 25.8] - [-7.5, 0, 20];
carCordInit = [13-1,0.5,-11];
car_noSMTCordInit = [13+2,0.5,-11+2];
carOrinentationInit = [0, 1, 0, -pi/2];
diffCamera = cameraCordInit - carCordInit;

car.translation = carCordInit;
car_noSMT.translation = car_noSMTCordInit;
camera.position = car.translation + diffCamera;
car.rotation    = carOrinentationInit;
car_noSMT.rotation    = carOrinentationInit;


world2 = vrworld('ugv_vr3.wrl');
open(world2);
fig2 = view(world2, '-internal');
vrdrawnow;

car2 = vrnode(world2, 'Automobile2');
car_noSMT2 = vrnode(world2, 'Automobile');
camera2 = vrnode(world2, 'View1');


camera2CordInit = [2.4, 10.2, 25.8] - [-7.5, 0, 20];
car2CordInit = carCordInit;
car2OrinentationInit = carOrinentationInit;
diffCamera2 = camera2CordInit - car2CordInit;

car2.translation = car2CordInit;
car_noSMT2.translation = car_noSMTCordInit;
camera2.position = car2.translation + diffCamera2;
car2.rotation    = car2OrinentationInit;
car_noSMT2.rotation    = car2OrinentationInit;

vrdrawnow;



%% Robot parameters

% Motor Parameters
Lm          = 1E-3;         % Motor Inducatnce
Rm            = 3.1;     
% Motor resistance
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


%% Define robot dynamics
n = 2;      p = 3;
A           = [0        1; 0    -Br/Mt];
B           = [0         ; 1/Mt      ];
C           = [1        0; 0        1;  0       1];   %GPS for x, Encoder for v
D           = [0         ; 0         ;  0       ];
Ts          = 0.01;
Sys         = ss(A,B,C,D);
sys_car     = c2d(Sys, Ts, 'ZOH');

tau         = n;
OO          = [];
for t = 1 : 1 : tau
    OO      = [OO sys_car.C*sys_car.A^(t-1)];
end

O_SMT       = [];
for counter = 1 : p
    O_SMT{counter}    = [reshape(OO(counter,:), n,tau)'];
end


%% Design Controller
Q = [1000 0; 0 1];
R = eye(1);
[K_lqr,S,e] = dlqr(sys_car.A,sys_car.B,Q,R,zeros(1)); % gain normal controller for in order

%%
attack_switch = 0;
stop_simulation = 0;

%% Simulate robot
points = [46,        91,      139,  184];
angles = [-pi/2,     pi,     pi/2,  0];

state       = [0;   0];
state_noSMT = state;

angle       = 0;
angle_noSMT = 0;

Fx              = K_lqr  *([points(1); 5] - state);
Fx_noSMT        = K_lqr  *([points(1); 5] - state_noSMT);
end_sim         = 0;

smt             = SMT();
noise_power     = 0.1;


plot_length = 200;
velocity_history        = zeros(1,plot_length);
velocity_hat_history    = zeros(1,plot_length);

velocity_noSMT_history  = zeros(1,plot_length);
velocity_hat_noSMT_history= zeros(1,plot_length);

attack_history          = zeros(1,plot_length);


time                    = 0:Ts:(plot_length-1)*Ts;
figure;
subplot(3,1,1);     
h1 = plot(time, velocity_history, 'YDataSource', 'velocity_history'); 
hold on; 
h11 =plot(time, velocity_hat_history, 'b:','YDataSource', 'velocity_hat_history'); 
title('Velocity of "safe" car');

subplot(3,1,2);     
h2 = plot(time, velocity_noSMT_history, 'r');
hold on;
h22 = plot(time, velocity_hat_noSMT_history, 'r:');  
title('Velocity of "unsafe" car');
subplot(3,1,3);     h3 = plot(time, attack_history, 'k');  title('Attack signal');

set(gcf,'currentcharacter','c')


while(end_sim == 0)
    %---- Update states according to dynamics
    state               = sys_car.A * state       + sys_car.B * Fx;
    state_noSMT         = sys_car.A * state_noSMT + sys_car.B * Fx_noSMT;
    
    %---- Prepare sensor measurements
    attack              = attack_switch(end)*100*[0 1 0; 0 1 0];
    noise               = noise_power*randn(2,3);
    
    %---- Estimate using SMT
    for mycounter = 1 : p
        Y_bar{mycounter} = O_SMT{mycounter}*state + attack(:,mycounter) + noise(:,mycounter); 
    end
    smt.init(n,p,tau,1);
    for counter = 1 : p
        smt.addSensorMeasurements(Y_bar{counter}, O_SMT{counter}, 4*noise_power, counter); %noisepower is multilplied by n*p to account for the overall noise over all sensors over all measurments per sensor
    end
    smt.markSensorAsSafe(1);
    [xhat, b] = smt.solve();
    
    %---- Estimate without using SMT
    for mycounter = 1 : p
        Y_bar_noSMT{mycounter} = O_SMT{mycounter}*state_noSMT + attack(:,mycounter)+ noise(:,mycounter);
    end
    xhat_noSMT= pinv([O_SMT{1}; O_SMT{2}; O_SMT{3}])*[Y_bar_noSMT{1}; Y_bar_noSMT{2}; Y_bar_noSMT{3}];
    
    
    %---- Control the vehicle (using SMT)
    if(state(1) <= points(1))
        car.translation     = carCordInit + [0 0 state(1)];
        car.rotation        = [0, 1, 0, angles(1)];
        
        car2.translation    = car.translation;
        car2.rotation       = car.rotation;
        
        Fx                  = K_lqr  *([points(1); 5] - xhat);
        phase = 1
    elseif(state(1) <= points(2))
        car.translation = carCordInit + [0 0 points(1)] - [state(1) - points(1) 0 0];
        car.rotation    = [0, 1, 0, angles(2)];
        
        car2.translation    = car.translation;
        car2.rotation       = car.rotation;
        
        Fx              = K_lqr  *([points(2); 5] - xhat);
        phase = 2
    elseif(state(1) <= points(3))
        car.translation = carCordInit + [points(1) - points(2) 0 points(1)] - [0 0 state(1) - points(2)];
        car.rotation    = [0, 1, 0, angles(3)];
        
        car2.translation    = car.translation;
        car2.rotation       = car.rotation;
        
        Fx              = K_lqr  *([points(3); 5] - xhat);
        phase = 3
    elseif(state(1) <= points(4))
        car.translation = carCordInit + [points(1) - points(2) 0 points(1) - points(3) + points(2)] + [state(1) - points(3) 0 0];
        car.rotation    = [0, 1, 0, angles(4)];
        
        car2.translation    = car.translation;
        car2.rotation       = car.rotation;
        
        Fx              = K_lqr  *([points(4); 5] - xhat);
        phase = 4
    else
        state(1)       = 0;
        
        Fx              = K_lqr  *([points(1); 5] - state);

        car.translation     = carCordInit + [0 0 state(1)];
        car.rotation        = [0, 1, 0, angles(1)];
        
        car2.translation    = car.translation;
        car2.rotation       = car.rotation;        
    end
    
    %---- Control the vehicle (without SMT)
    if(state_noSMT(1) <= points(1))
        car_noSMT.translation     = car_noSMTCordInit + [0 0 state_noSMT(1)];
        car_noSMT.rotation        = [0, 1, 0, angles(1)];
        
        car_noSMT2.translation    = car_noSMT.translation;
        car_noSMT2.rotation       = car_noSMT.rotation;
        
        Fx_noSMT                  = K_lqr  *([points(1); 5] - xhat_noSMT);
        phase_noSMT = 1
    elseif(state_noSMT(1) <= points(2))
        car_noSMT.translation = car_noSMTCordInit + [0 0 points(1)] - [state_noSMT(1) - points(1) 0 0];
        car_noSMT.rotation    = [0, 1, 0, angles(2)];
        
        car_noSMT2.translation    = car_noSMT.translation;
        car_noSMT2.rotation       = car_noSMT.rotation;
        
        Fx_noSMT              = K_lqr  *([points(2); 5] - xhat_noSMT);
        phase_noSMT = 2
    elseif(state_noSMT(1) <= points(3))
        car_noSMT.translation = car_noSMTCordInit + [points(1) - points(2) 0 points(1)] - [0 0 state_noSMT(1) - points(2)];
        car_noSMT.rotation    = [0, 1, 0, angles(3)];
        
        car_noSMT2.translation    = car_noSMT.translation;
        car_noSMT2.rotation       = car_noSMT.rotation;
        
        Fx_noSMT              = K_lqr  *([points(3); 5] - xhat_noSMT);
        phase_noSMT = 3
    elseif(state_noSMT(1) <= points(4))
        car_noSMT.translation = car_noSMTCordInit + [points(1) - points(2) 0 points(1) - points(3) + points(2)] + [state_noSMT(1) - points(3) 0 0];
        car_noSMT.rotation    = [0, 1, 0, angles(4)];
        
        car_noSMT2.translation    = car_noSMT.translation;
        car_noSMT2.rotation       = car_noSMT.rotation;
        
        Fx_noSMT              = K_lqr  *([points(4); 5] - xhat_noSMT);
        phase_noSMT = 4
    else
        state_noSMT(1) = 0;
        Fx_noSMT        = K_lqr  *([points(1); 5] - state_noSMT);
        
        car_noSMT.translation     = car_noSMTCordInit + [0 0 state_noSMT(1)];
        car_noSMT.rotation        = [0, 1, 0, angles(1)];
        
        car_noSMT2.translation    = car_noSMT.translation;
        car_noSMT2.rotation       = car_noSMT.rotation;
    end
    
    
    camera.position = car.translation + diffCamera;
    camera2.position = car_noSMT.translation + diffCamera2;
    vrdrawnow;
    
    time                    = [time(2:end) time(end)+Ts];
    
    velocity_history        = [velocity_history(2:end) state(2)];
    velocity_hat_history    = [velocity_hat_history(2:end) xhat(2)];
    
    set(h1,'YData',velocity_history);
    set(h11,'YData',velocity_hat_history);
    
    
    velocity_noSMT_history        = [velocity_noSMT_history(2:end) state_noSMT(2)];
    velocity_hat_noSMT_history    = [velocity_hat_noSMT_history(2:end) xhat_noSMT(2)];
    set(h2,'YData',velocity_noSMT_history);
    set(h22,'YData',velocity_hat_noSMT_history);
    
    attack_history          = [attack_history(2:end) attack(2,2)];
    set(h3,'YData',attack_history);
    drawnow; 
    
    
    
    
    
    pause(0.1);
end
%%
break
z1 = -10:35;
x1 = 13 * ones(size(z1));
y1 = 0.1* ones(size(z1));



x2 = 13:-1:-31;
z2 = 35 * ones(size(x2));
y2 = 0.1* ones(size(z2));

z3 = 35:-1:-10;
x3 = -31 * ones(size(z3));
y3 = 0.1* ones(size(z3));



x4 = -31:1:13;
z4 = -11 * ones(size(x4));
y4 = 0.1* ones(size(z4));

for i=1:length(x1)
    %car.translation = [x1(i) y1(i) z1(i)];
    car.translation = car.translation + [0 0 1];
    camera.position = car.translation + diffCamera;
    vrdrawnow;
    pause(0.1);
end

car.rotation = [0, 1, 0, pi];
vrdrawnow;



for i=1:length(x2)
    car.translation = [x2(i) y2(i) z2(i)];
    camera.position = car.translation + diffCamera;
    vrdrawnow;
    pause(0.1);
end

car.rotation = [0, 1, 0, pi/2];
vrdrawnow;


for i=1:length(x3)
    car.translation = [x3(i) y3(i) z3(i)];
    camera.position = car.translation + diffCamera;
    vrdrawnow;
    pause(0.1);
end

car.rotation = [0, 1, 0, 0];
vrdrawnow;


for i=1:length(x4)
    car.translation = [x4(i) y4(i) z4(i)];
    camera.position = car.translation + diffCamera;
    vrdrawnow;
    pause(0.1);
end













%%
% edit(vrworld('my_vrmount.wrl'))


% %  output                    = subs(h, [x,y,theta], [state(1:3)']); % measurment
% %  outputHistory(:,counter)  = output;   % save measurment
% %  attack                    = [zeros(p-1,1); randn(1)];
% %  attackedOutputHistory(:,counter)     = output + attack;
%   %stateHistory(:,counter)   = state;   % save current state
%   if(norm(input(1) - input(2))>1E-5)
%       state                     = subs(f, [x, y, theta, v_r, v_l], [state(1:3)', input']);  % update process
%   else
%       state                     = subs(f_linear, [x, y, theta, v_r, v_l], [state(1:3)', input']);  % update process
%   end
%   %outputs                     = subs(h, [x,y,theta], f_current);
