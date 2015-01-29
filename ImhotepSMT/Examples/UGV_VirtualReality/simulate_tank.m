clc
close all
clear all

%% Prepare system modelr
tank_model_prepare;

%% Prepare attack vector
load 'attack_encoder'
attack_size                                         = size(Attack_Left_encoder.signals.values',2);
time_attack                                         = 0 : 0.1 : (attack_size-1)*0.1;
time_attack_interp                                  = 0 : 0.01 : (attack_size-1)*0.1;
attack                                              = [zeros(size(time_attack_interp)); ...
                                                        interp1(time_attack, Attack_Left_encoder.signals.values, time_attack_interp); ...
                                                        interp1(time_attack, Attack_Right_encoder.signals.values, time_attack_interp)];

attack = 10*attack;
%% Initialize robot estimate
x_linear                                            = zeros(n,1);
x_rotation                                          = zeros(n,1);

%% Initialize the state estimate
z_hat                                               = zeros(n+n*p,1);
linear_state                                        = z_hat(1:n);
rotation_state                                      = zeros(n,1);

%% Initialize data collectros
X_linear    = [];
Fx_hist     = [];
Xhat_linear = [];
Khat_linear = [];
xhat = [0 ; 0];
Khat = [0 ; 0; 0];
K = [0 ; 0; 0];
KK = [];
%% Simulate
t = 1; Fx = 0; Torque = 0;
noise_power = 0; %0.1;
[set_point_linear, set_point_rotation, end_sim]     = path_controller(linear_state, rotation_state);

smt = SMT();


while(end_sim == 0)
%while(t < 89)
    % Collect Responses
    X_linear                                        = [X_linear; x_linear'];
    
    % Generate outputs and Estimate using ETPL
    if  t > n
        t
        tt = t - 1;
        x                                               = X_linear(tt-n+1,:)';
        E                                               = attack(:,tt-n+1:tt)';
        for mycounter = 1 : p
            Y_bar{mycounter} = O_SMT{mycounter}*x + 20*E(:,mycounter);
%            Y_bar{mycounter} = Y_bar{mycounter} + noise_power*randn(length(Y_bar{mycounter}),1);
        end
%        Y_bar                                           = C_bar*z;
%        Y_bar                                           = Y_bar + noise_power*randn(length(Y_bar),1);
%         
%         tt = t;
%         u                                               = [Fx_hist(tt-2); Fx_hist(tt-1)];
%         z_plus                                          = ([X_linear(tt-n+1,:)'; reshape(attack(:,tt-n+1:tt),n*p,1)]);      
%         
%         tt = t+1;
%         z_plus_plus                                     = ([X_linear(tt-n+1,:)'; reshape(attack(:,tt-n+1:tt),n*p,1)]);      
%         Y_bar_plus                                      = C_bar*z_plus_plus;
%         Y_bar_plus                                      = Y_bar_plus + noise_power*randn(length(Y_bar_plus),1);
%         
%         y_ahead                                         = Y_bar_plus(1:p);
%         
        
        
%        tic
%        [ z_hat, attacekd_sensors_etpl, trigger ]       = EventTriggeredLeunberger_vector( Y_bar, y_ahead, Y_bar_plus, u, sys_bar, z_hat, 1, alpha_I, n, p );
%        [xhat, a_hat, index_attacked_sensors, V_history_grad, iter, success] = modified_iterative_hard_thresholding_dynamic(Y_bar, u, SysD, z_hat, 1E-6, 1, IN); z_hat = [xhat; a_hat];
%       time_etpl = toc;
%        V_etpl(t) = norm(z_plus - z_hat);

        smt.init(n,p,tau,1);
        for counter = 1 : p
            smt.addSensorMeasurements(Y_bar{counter}, O_SMT{counter}, 4*noise_power, counter); %noisepower is multilplied by n*p to account for the overall noise over all sensors over all measurments per sensor
        end
        smt.markSensorAsSafe(1);

        if(t == 249)
            'hop'
        end
        [xhat, Khatt] = smt.solve();
    %numberOfTheoryCalls = smt.mumberOfTheoryCalls;
%     xhat = x_linear + noise_power*randn(length(xhat),1);
%   
        if(isempty(xhat) == 0)
             Khat = [0;0;0];
             Khat(Khatt) = 1;
             K = [0; norm(E(:,2)) > 0; norm(E(:,3)) > 0];
%              K' 
%              Khat'
%              error= norm(xhat - X_linear(t-n,:)')
        end
    end
    Xhat_linear(t,:)                                  = [xhat'];
    Khat_linear(t,:)                                  = [Khat'];
    KK(t,:)                                           = [K'];
    
    linear_state                                    = x_linear;
    rotation_state                                  = x_rotation;
    
    % Generate Input
    Fx                                              = K_linear  *(set_point_linear - linear_state);
    Torque                                          = K_rotation*(set_point_rotation - rotation_state);
    
    Fx_hist                                         = [Fx_hist; Fx];
    
    % Apply inputs and update states
    x_linear                                        = sys_linear.A * x_linear       + sys_linear.B * Fx;
    x_rotation                                      = sys_rotation.A * x_rotation   + sys_rotation.B * Torque;
    
    % Generate Set point
    [set_point_linear, set_point_rotation, end_sim] = path_controller(linear_state, rotation_state);
    
    % advance simulation clock
    t                                               = t + 1;
    
%    save 'results_smt'
end

%% Plot Results
close all
timeX = 0 : Ts : (length(X_linear)-1)*Ts;
timeXhat = 0 : Ts : (length(Xhat_linear)-1)*Ts;

figure; plot(timeX, X_linear(:,1)); hold on; plot(timeXhat, Xhat_linear(:,1), 'r');
figure; plot(timeX, X_linear(:,2)); hold on; plot(timeXhat, Xhat_linear(:,2), 'r');

figure; 
subplot(2,1,1); plot(KK(:,1)'); subplot(2,1,2); plot(Khat_linear(:,1), 'r');
figure; 
subplot(2,1,1); plot(KK(:,2)'); subplot(2,1,2); plot(Khat_linear(:,2), 'r');
figure; 
subplot(2,1,1); plot(KK(:,3)'); subplot(2,1,2); plot(Khat_linear(:,3), 'r');
break
%%

%%
time = 0 : Ts : (length(Xhat_linear)-1)*Ts;

data = [time' X_linear(:,1)];
save('x_tank_smt.txt', 'data','-ascii','-double')

data = [time' X_linear(:,2)];
save('v_tank_smt.txt', 'data','-ascii','-double')

data = [time' attack(2,1:length(time))'];
save('a1_tank_smt.txt', 'data','-ascii','-double')

data = [time' attack(3,1:length(time))'];
save('a2_tank_smt.txt', 'data','-ascii','-double')



data = [time' Xhat_linear(:,1)];
save('xhat_smt.txt', 'data','-ascii','-double')

data = [time' Xhat_linear(:,2)];
save('vhat_smt.txt', 'data','-ascii','-double')

data = [time' Khat_linear(:,1)];
save('support_hat1_smt.txt', 'data','-ascii','-double')

data = [time' Khat_linear(:,2)];
save('support_hat2_smt.txt', 'data','-ascii','-double')


break
%%
time = 0 : Ts : (length(Xhat_linear)-1)*Ts;

data = [time' X_linear(:,1)];
save('x_tank_etpl.txt', 'data','-ascii','-double')

data = [time' X_linear(:,2)];
save('v_tank_etpl.txt', 'data','-ascii','-double')

data = [time' attack(2,1:length(time))'];
save('a1_tank_etpl.txt', 'data','-ascii','-double')

data = [time' attack(3,1:length(time))'];
save('a2_tank_etpl.txt', 'data','-ascii','-double')





data = [time' Xhat_linear(:,1)];
save('xhat_etpl.txt', 'data','-ascii','-double')

data = [time' Xhat_linear(:,2)];
save('vhat_etpl.txt', 'data','-ascii','-double')

data = [time' Ahat_linear(:,2)];
save('ahat1_etpl.txt', 'data','-ascii','-double')

data = [time' Ahat_linear(:,3)];
save('ahat2_etpl.txt', 'data','-ascii','-double')
