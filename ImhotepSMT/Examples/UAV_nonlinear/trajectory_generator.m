function [ desired_state ] = trajectory_generator(t, qn, map, path)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
% Author: Yanwei Du, MEAM 620 Spring 2014, University of Pennsylvania
% Date: February 2014

persistent coeff_x coeff_y coeff_z num time
persistent start stop t_total

v_max = 1.0; % v < 5.5

if nargin > 2
    validPoint = path{1};
    start = validPoint(1:end-1, :);
    stop = validPoint(2:end, :);
    % calculate distance
    dis = sqrt(sum((stop - start).^2, 2));
    num = size(start, 1);
    dis_total = sum(dis);
    t_total = dis_total/v_max;
    t_sum = 0;
    % calculate coefficient
    coeff_x = zeros(6, num);
    coeff_y = coeff_x;
    coeff_z = coeff_x;
      
    for i = 1 : num
        
        p0 = start(i, :);
        pt = stop(i, :);
        v0 = 0;
        vt = 0;
        a0 = 0;
        at = 0;
        
        t = ceil(sqrt(dis(i)/dis_total)*t_total);
        time(i) = t_sum;
        t_sum = t_sum + t;
        X = [0,      0,      0,     0,   0   1;
             t^5,    t^4,    t^3,   t^2, t,  1;
             0,      0,      0,     0,   1,  0;
             5*t^4,  4*t^3,  3*t^2, 2*t, 1,  0;
             0,      0,      0,     2,   0,  0;
             20*t^3, 12*t^2, 6*t,   2,   0,  0];

        coeff_x(:, i) = X \ [p0(1), pt(1), v0, vt, a0, at]';
        coeff_y(:, i) = X \ [p0(2), pt(2), v0, vt, a0, at]';
        coeff_z(:, i) = X \ [p0(3), pt(3), v0, vt, a0, at]';
    end
    time(i + 1) = ceil(t_sum);
    return;   
end  


if t >= time(end)
    pos = stop(end,:)';
    vel = [0; 0; 0];
    acc = [0; 0; 0];
else

    idx = find(time<=t);
    idx = idx(end);
    t = t - time(idx);
    if idx > size(coeff_x, 2)
        idx = size(coeff_x, 2);
    end

    s = [t^5, t^4, t^3, t^2, t, 1];
    sd = [5*t^4, 4*t^3, 3*t^2, 2*t, 1, 0];
    sdd = [20*t^3, 12*t^2, 6*t, 2, 0, 0];


    px = s * coeff_x(:, idx);
    vx = sd * coeff_x(:, idx);
    ax = sdd * coeff_x(:, idx);

    py = s * coeff_y(:, idx);
    vy = sd * coeff_y(:, idx);
    ay = sdd * coeff_y(:, idx);

    pz = s * coeff_z(:, idx);
    vz = sd * coeff_z(:, idx);
    az = sdd * coeff_z(:, idx);

    pos = [px; py; pz];
    vel = [vx; vy; vz];
    acc = [ax; ay; az];
end

yaw = 0;
yawdot = 0;
desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;
    
    
    
    

end

