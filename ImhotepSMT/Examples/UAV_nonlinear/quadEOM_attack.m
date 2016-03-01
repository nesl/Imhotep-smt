function [tsave, xsave, xhat, attack, execTime, attackedSensor] = quadEOM_attack(t, s, qn, controlhandle, trajhandle, params, attack_flag, SMT_flag)
% QUADEOM Wrapper function for solving quadrotor equation of motion
% 	quadEOM takes in time, state vector, controller, trajectory generator
% 	and parameters and output the derivative of the state vector, the
% 	actual calcution is done in quadEOM_readonly.
%
% INPUTS:
% t             - 1 x 1, time
% s             - 13 x 1, state vector = [x, y, z, xd, yd, zd, qw, qx, qy, qz, p, q, r]
% qn            - quad number (used for multi-robot simulations)
% controlhandle - function handle of your controller
% trajhandle    - function handle of your trajectory generator
% params        - struct, output from nanoplus() and whatever parameters you want to pass in
%
% OUTPUTS:
% sdot          - 13 x 1, derivative of state vector s
%
% NOTE: You should not modify this function
% See Also: quadEOM_readonly, nanoplus

global F M

% convert state to quad stuct for control
qd{qn} = stateToQd(s);

% Get desired_state
%desired_state = trajhandle(t, qn);
desired_state = trajhandle(t(1), qn);

% The desired_state is set in the trajectory generator
qd{qn}.pos_des      = desired_state.pos;
qd{qn}.vel_des      = desired_state.vel;
qd{qn}.acc_des      = desired_state.acc;
qd{qn}.yaw_des      = desired_state.yaw;
qd{qn}.yawdot_des   = desired_state.yawdot;

% attack
attack          = zeros(3,1);


%qd{qn}.pos_des

%[qd{qn}.pos_des(1) == 2.5, qd{qn}.pos_des(2) < 2.5, qd{qn}.pos_des(2) > 0.0]
if attack_flag == 1
    if(qd{qn}.pos_des(2) == 0 && qd{qn}.pos_des(1) < 2.5 && qd{qn}.pos_des(1) > 0.0)
        attack(3)       = 1.058*sin(3*pi*t(1));
    elseif(qd{qn}.pos_des(1) > 2.49 && qd{qn}.pos_des(2) < 2.5 && qd{qn}.pos_des(2) > 0.0)
        attack(3) = 0;
    elseif(qd{qn}.pos_des(2) > 2.49 && qd{qn}.pos_des(1) < 2.4 && qd{qn}.pos_des(1) > 0)
        attack(3)       = 1.2*sin(3*pi*t(1));
    elseif(qd{qn}.pos_des(1) == 0 && qd{qn}.pos_des(2) < 2.5 && qd{qn}.pos_des(2) > 0)
        attack(3) = 0;
    else
        attack(3)       = 0;
    end
end


if(abs(qd{qn}.vel(3)) > 3 )
    attack(3) = 0;
end
    
qd{qn}.vel      = qd{qn}.vel + attack;

global smt uav

execTime = 0;
attackedSensor = 0;

if(SMT_flag)
    time = tic;
    [shat, attackedSensor, status] = smt.addInputsOutputs([F;M], [qd{qn}.pos; qd{qn}.vel; qd{qn}.euler; qd{qn}.omega]);
    execTime                = toc(time);
    
    if(isempty(attackedSensor))
        attackedSensor = 0;
    end
    

    % qd{qn}.pos
    % qd{qn}.vel

    if(status == 1)
        qd_hat{qn}              = stateToQd(shat);
        qd_hat{qn}.pos_des      = qd{qn}.pos_des;
        qd_hat{qn}.vel_des      = qd{qn}.vel_des;
        qd_hat{qn}.acc_des      = qd{qn}.acc_des;
        qd_hat{qn}.yaw_des      = qd{qn}.yaw_des;
        qd_hat{qn}.yawdot_des   = qd{qn}.yawdot_des;
    else
        qd_hat = qd;
    end
else
    qd_hat = qd;
end


xhat               = [qd_hat{qn}.pos; qd_hat{qn}.vel; RotToQuat(RPYtoRot_ZXY(qd_hat{qn}.euler(1),qd_hat{qn}.euler(2),qd_hat{qn}.euler(3))); qd_hat{qn}.omega];
% get control outputs
[F, M, trpy, drpy] = controlhandle(qd_hat, t, qn, params);

% compute derivative
%sdot = quadEOM_readonly(t, s, F, M, params);

[tsave, xsave] = ode45(@(t,s) quadEOM_readonly(t, s, F, M, params), t, s);

end
