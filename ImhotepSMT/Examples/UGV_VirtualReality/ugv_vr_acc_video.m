%% Book  keeping
clc; close all; rand('state',0); randn('state',0);

%% Initialize 3D model
world = vrworld('ugv_vr2.wrl');
open(world);

if(exist('fig') == 0)
    fig = view(world, '-internal');
end
vrdrawnow;

car = vrnode(world, 'Automobile2');
carCordInit = [13,0.5,-11];

car_SMT = vrnode(world, 'Automobile');

camera = vrnode(world, 'View1');
cameraCordInit = [2.4, 10.2, 25.8] - [-7.5, 0, 20];
diffCamera = cameraCordInit - carCordInit;


world2 = vrworld('ugv_vr3.wrl');
open(world2);
if(exist('fig2') == 0)
    fig2 = view(world2, '-internal');
end


car2 = vrnode(world2, 'Automobile2');
car_noSMT = vrnode(world2, 'Automobile');

camera2 = vrnode(world2, 'View1');
camera2CordInit     = cameraCordInit;
diffCamera2         = diffCamera;

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
Q = [500 0; 0 1];
R = eye(1);
[K_lqr,S,e] = dlqr(sys_car.A,sys_car.B,Q,R,zeros(1)); % gain normal controller for in order

%%
attack_switch = 0;
stop_simulation = 0;
pause_sim = 0;

%% Prepare velocity plot
plot_length = 200;
velocity_SMT_history        = zeros(1,plot_length);
velocity_hat_SMT_history    = zeros(1,plot_length);

velocity_noSMT_history  = zeros(1,plot_length);
velocity_hat_noSMT_history= zeros(1,plot_length);

attack_history          = zeros(1,plot_length);

time                    = 0:Ts:(plot_length-1)*Ts;
figure;

subplot(3,1,1);     
h1 = plot(time, velocity_SMT_history, 'YDataSource', 'velocity_history'); 
hold on; 
h11 =plot(time, velocity_hat_SMT_history, 'b:','YDataSource', 'velocity_hat_history'); 
axis([0,2,0, 110])
title('Velocity (solid) and estimated velocity (dashed) of the blue car (using secure estimator)','FontSize',18);

subplot(3,1,2);     
h2 = plot(time, velocity_noSMT_history, 'r');
hold on;
h22 = plot(time, velocity_hat_noSMT_history, 'r:');  
title('Velocity (solid) and estimated velocity (dashed) of the red car (without secure estimator)','FontSize',18);
axis([0,2,0, 110])

subplot(3,1,3);     h3 = plot(time, attack_history, 'k');  
title('Attack signal','FontSize',18);
axis([0,2,-100, 100])

set(gcf,'currentcharacter','c')

%% Simulate robot
points = [46,        91,      137,  182];
angles = [-pi/2,     pi,     pi/2,  0];


initShift = 7;
state       = [initShift+5;   0];
state_SMT   = [2;   0];
state_noSMT = [2;   0];

angle       = 0;
angle_noSMT = 0;

Fx              = K_lqr  *([points(1); 5] - state);
Fx_SMT          = K_lqr  *(state - [initShift;0] - state_SMT);
Fx_noSMT        = K_lqr  *(state - [initShift;0] - state_noSMT);
end_sim         = 0;

smt             = SMT();
noise_power     = 0.1;

time_counter = 0;


while(end_sim == 0)
    if(state       == [initShift+5;   0])
        pause;
        pause(3.0);
    end

    if(pause_sim == 1)
        pause(0.1);
        continue;
    end
    
    %time_counter = time_counter + 1;
    
%     if(time_counter == 300)
%         attack_switch = 1;
%     end
    %---- Update the state of the three cars according to dynamics
    state               = sys_car.A * state       + sys_car.B * Fx;
    state_SMT           = sys_car.A * state_SMT   + sys_car.B * Fx_SMT;
    state_noSMT         = sys_car.A * state_noSMT + sys_car.B * Fx_noSMT;
    
    %---- Prepare sensor measurements
    attack              = attack_switch(end)*-100*[0 1 0; 0 1 0];
    noise               = noise_power*randn(2,3);
    
    %========== Leader Car ==========
    state(1) = mod(state(1),points(4));
    if(state(1) <= points(1))
        car.translation     = carCordInit + [0 0 state(1)];
        car.rotation        = [0, 1, 0, angles(1)];
        
        car2.translation    = car.translation;
        car2.rotation       = car.rotation;
        
        Fx                  = K_lqr  *([points(1); 5] - state);
        phase = 1;
    elseif(state(1) <= points(2))
        car.translation = carCordInit + [0 0 points(1)] - [state(1) - points(1) 0 0];
        car.rotation    = [0, 1, 0, angles(2)];
        
        car2.translation    = car.translation;
        car2.rotation       = car.rotation;
        
        Fx              = K_lqr  *([points(2); 5] - state);
        phase = 2;
        
    elseif(state(1) <= points(3))
        car.translation = carCordInit + [points(1) - points(2) 0 points(1)] - [0 0 state(1) - points(2)];
        car.rotation    = [0, 1, 0, angles(3)];
        
        car2.translation    = car.translation;
        car2.rotation       = car.rotation;
        
        Fx              = K_lqr  *([points(3); 5] - state);
        phase = 3;
        
    elseif(state(1) <= points(4))
        
        car.translation = carCordInit + [points(1) - points(2) 0 points(1) - points(3) + points(2)] + [state(1) - points(3) 0 0];
        car.rotation    = [0, 1, 0, angles(4)];
        
        car2.translation    = car.translation;
        car2.rotation       = car.rotation;
        
        Fx              = K_lqr  *([points(4); 5] - state);
        phase = 4;        
    end
    
    %========== Follower Car using SMT ==========
    %-- Estimate using SMT
    for mycounter = 1 : p
        Y_bar{mycounter} = O_SMT{mycounter}*state_SMT + attack(:,mycounter) + noise(:,mycounter); 
    end
    smt.init(n,p,tau,1);
    for counter = 1 : p
        smt.addSensorMeasurements(Y_bar{counter}, O_SMT{counter}, 4*noise_power, counter); %noisepower is multilplied by n*p to account for the overall noise over all sensors over all measurments per sensor
    end
    smt.markSensorAsSafe(1);
    [xhat_SMT, b] = smt.solve();
    %-- Control the vehcile
    state_SMT(1)    = mod(state_SMT(1),points(4));
    xhat_SMT(1)     = mod(xhat_SMT(1),points(4));
    if(state_SMT(1) <= points(1))
        
        car_SMT.translation       = carCordInit + [0 0 state_SMT(1)];
        car_SMT.rotation          = [0, 1, 0, angles(1)];
        
        error                     = state - [initShift;0] - xhat_SMT;
        Fx_SMT                    = K_lqr  *error;
        phase_SMT = 1;
    elseif(state_SMT(1) <= points(2))
        car_SMT.translation       = carCordInit + [0 0 points(1)] - [state_SMT(1) - points(1) 0 0];
        car_SMT.rotation          = [0, 1, 0, angles(2)];
        
        error                     = state - [initShift;0] - xhat_SMT;
        Fx_SMT                    = K_lqr  *error;
        
        phase_SMT = 2;
    elseif(state_SMT(1) <= points(3))
        car_SMT.translation = carCordInit + [points(1) - points(2) 0 points(1)] - [0 0 state_SMT(1) - points(2)];
        car_SMT.rotation    = [0, 1, 0, angles(3)];
        
        error                     = state - [initShift;0] - xhat_SMT;
        Fx_SMT                    = K_lqr  *error;
        phase_SMT = 3;
    elseif(state_SMT(1) <= points(4))
        car_SMT.translation = carCordInit + [points(1) - points(2) 0 points(1) - points(3) + points(2)] + [state_SMT(1) - points(3) 0 0];
        car_SMT.rotation    = [0, 1, 0, angles(4)];
        if(phase > 1)
            error                     = state - [initShift;0] - xhat_SMT;
        else
            error                     = [initShift - state(1);state(2)]  - [points(4) - xhat_SMT(1) ;xhat_SMT(2)];
        end
        Fx_SMT                        = K_lqr  *error;
        phase_SMT = 4;
    end
    
    %========== Follower Car not using SMT ==========
    %-- Estimate without using SMT
    for mycounter = 1 : p
        Y_bar_noSMT{mycounter} = O_SMT{mycounter}*state_noSMT + attack(:,mycounter)+ noise(:,mycounter);
    end
    xhat_noSMT= pinv([O_SMT{1}; O_SMT{2}; O_SMT{3}])*[Y_bar_noSMT{1}; Y_bar_noSMT{2}; Y_bar_noSMT{3}];
    
    %---- Control the vehicle
    state_noSMT(1) = mod(state_noSMT(1),points(4));
    xhat_noSMT(1) = mod(xhat_noSMT(1),points(4));
    
    if(state_noSMT(1) <= points(1))
        car_noSMT.translation     = carCordInit + [0 0 state_noSMT(1)];
        car_noSMT.rotation        = [0, 1, 0, angles(1)];
        
        error                     = state - [initShift;0] - xhat_noSMT;
        Fx_noSMT                  = K_lqr  *error;
        phase_noSMT = 1;
    elseif(state_noSMT(1) <= points(2))
        if(time_counter < 1)
            pause
            pause(3.0)
        end
        
        if(time_counter < 17)
            time_counter = time_counter + 1;
            attack_switch = 1;
        else
            attack_switch = 0;
        end
        
        car_noSMT.translation = carCordInit + [0 0 points(1)] - [state_noSMT(1) - points(1) 0 0];
        car_noSMT.rotation    = [0, 1, 0, angles(2)];
        
        error                     = state - [initShift;0] - xhat_noSMT;
        Fx_noSMT                  = K_lqr  *error;
        
        phase_noSMT = 2;
    elseif(state_noSMT(1) <= points(3))
        if(phase_noSMT == 2)
            pause
            pause(3.0)
        end
        attack_switch = 1;
        
        car_noSMT.translation = carCordInit + [points(1) - points(2) 0 points(1)] - [0 0 state_noSMT(1) - points(2)];
        car_noSMT.rotation    = [0, 1, 0, angles(3)];
        
        error                     = state - [initShift;0] - xhat_noSMT;
        Fx_noSMT                  = K_lqr  *error;
        phase_noSMT = 3;
    elseif(state_noSMT(1) <= points(4))
        
        
        car_noSMT.translation = carCordInit + [points(1) - points(2) 0 points(1) - points(3) + points(2)] + [state_noSMT(1) - points(3) 0 0];
        car_noSMT.rotation    = [0, 1, 0, angles(4)];
        
        if(phase > 1)
            error                     = state - [initShift;0] - xhat_noSMT;
        else
            error                     = [initShift - state(1);state(2)]  - [points(4) - xhat_noSMT(1) ;xhat_noSMT(2)];
        end
        Fx_noSMT                  = K_lqr  *error;
        phase_noSMT = 4;
    end
    
    %======== Detect crash ===========
    norm_error = norm(state(1) - state_noSMT(1));
    if(norm(state(1) - state_noSMT(1)) < 1)
        pause(5.0);
        attack_switch = 0;
        state       = [initShift+5;   0];
        state_SMT   = [2;   0];
        state_noSMT = [2;   0];
    end
    
    %======== Update the Scene ==========
    camera.position  = car.translation + diffCamera;
    camera2.position = car2.translation + diffCamera2;
    vrdrawnow;
    
    time                        = [time(2:end) time(end)+Ts];
    
    velocity_SMT_history        = [velocity_SMT_history(2:end) state_SMT(2)];
    velocity_hat_SMT_history    = [velocity_hat_SMT_history(2:end) xhat_SMT(2)];
    
    set(h1,'YData',velocity_SMT_history);
    set(h11,'YData',velocity_hat_SMT_history);
    
    
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
