function [ quad ] = quad_pos( pos, rot, arm_mat, H )
%QUAD_POS Calculates coordinates of quadrotor's position in world frame
% pos       3x1 position vector [x; y; z];
% rot       3x3 body-to-world rotation matrix
% L         1x1 length of the quad

if nargin < 4; H = 0.05; end

% wRb   = RPYtoRot_ZXY(euler(1), euler(2), euler(3))';
wHb   = [rot pos(:); 0 0 0 1]; % homogeneous transformation from body to world



l1x = arm_mat(1,1);  l1y = arm_mat(1,2);
l2x = arm_mat(2,1);  l2y = arm_mat(2,2);
l3x = arm_mat(3,1);  l3y = arm_mat(3,2);
l4x = arm_mat(4,1);  l4y = arm_mat(4,2);
quadBodyFrame  = [l1y,   l1x,   0,   1; 
                  -l2y,   l2x,    0,   1;
                  -l3y,  -l3x,    0,   1; 
                   l4y,  -l4x,   0    1; 
                   0,     0,     0,   1; 
                   0,     0,     H,   1]';

% quadBodyFrame  = [L 0 0 1; 0 L 0 1; -L 0 0 1; 0 -L 0 1; 0 0 0 1; 0 0 H 1]';
quadWorldFrame = wHb * quadBodyFrame;
quad           = quadWorldFrame(1:3, :);

end
