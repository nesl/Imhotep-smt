function state = add_noise( s )
% ADD_NOISE

state = s;
pos = s(1:3); % position
vel = s(4:6); % velocity

% Gaussian Noise
state(1:3) = pos + normrnd(0, 0.002, 3, 1);
state(4:6) = vel + normrnd(0, 0.002, 3, 1);


end

