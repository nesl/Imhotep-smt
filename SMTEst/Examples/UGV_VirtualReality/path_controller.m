function [set_point_linear, set_point_rotation, phase, end_sim] = path_controller(linear_state, rotation_state)

persistent phase_old;
persistent counter;
if isempty(phase_old) 
    phase_old = int32(0);
end

if isempty(counter) 
    counter = int32(0);
end

% theta = theta + pi;

x = linear_state(1);
% v = states(2);
theta = rotation_state(1);

% K = [ 0.999439303200728         0.896802567284867];
% T = -3;


points = [45,        45,      90,         90,   135,   135, 180 , 180];
angles = [-pi/2,     pi,      pi,       pi/2, pi/2,   0, 0, 0];
allowance_theta = pi/100;
allowance_x = 0.1;

segment = fix(phase_old/2)+1;
phase = fix(rem(phase_old,4));
% [phase_old, segment, phase]
% theta

if(phase_old == 13)
    phase = int32(4);
end

% Fx          = 0;
% torque      = -0.5;
% phase_new   = phase_old;
% end_sim     = 0;
%          
set_point_linear = [points(segment);0];
set_point_rotation = [angles(segment);0];
switch phase
    case 0,  %move Fwd 
        error_x = set_point_linear(1) - x;
        if ( error_x <= allowance_x && error_x >= -1*allowance_x)
%             Fx          = K * [error_x;v];
%             torque      = 0;
            phase_new   = phase_old + int32(1);
            counter     = int32(0);
            end_sim     = 0;
        else
%             Fx          = K * [error_x;v];
%             torque      = 0;
            phase_new   = phase_old;
            end_sim     = 0;
            counter     = int32(0);
        end   
    case 1, %Stop for a while
        if (counter < 5 )
%             Fx          = 0;
%             torque      = 0;
            phase_new   = phase_old;
            end_sim     = 0;
            counter     = counter + 1;
        else
%             Fx          = 0;
%             torque      = 0;
            phase_new   = phase_old + int32(1);
            end_sim     = 0;
            counter     = int32(0);
        end
    case 2,  %turn right
        error_theta = set_point_rotation(1) - theta;
        if ( error_theta <= allowance_theta && error_theta >= -1*allowance_theta )
%             Fx          = 0;
%             torque      = 0;
            phase_new   = phase_old + int32(1);
            counter     = int32(0);
            end_sim     = 0;
        else
%             Fx          = 0;
%             torque      = -T;
            phase_new   = phase_old ;
            counter     = int32(0);
            end_sim     = 0;
        end
    case 3, %Stop for a while
        if (counter < 50 )
%             Fx          = 0;
%             torque      = 0;
            phase_new   = phase_old;
            end_sim     = 0;
            counter     = counter + 1;
        else
%             Fx          = 0;
%             torque      = 0;
            phase_new   = phase_old + int32(1);
            end_sim     = 0;
            counter     = int32(0);
        end
    otherwise
%         Fx          = 0;
%         torque      = 0;
        phase_new   = int32(13);
        end_sim     = 1;
        counter     = int32(0);
end

phase_old = phase_new;