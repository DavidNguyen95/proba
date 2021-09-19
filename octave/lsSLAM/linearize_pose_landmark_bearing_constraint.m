% Compute the error of a pose-landmark with bearing only constraint
% x 3x1 vector (x,y,theta) of the robot pose
% l 2x1 vector (x,y) of the landmark
% z 1x1 vector (theta) of the measurement, the position of the landmark in
%   the coordinate frame of the robot given by the vector x
%
% Output
% e 2x1 error of the constraint
% A 2x3 Jacobian wrt x
% B 2x2 Jacobian wrt l
function [e, A, B] = linearize_pose_landmark_bearing_constraint(x, l, z)

  % TODO compute the error and the Jacobians of the error
 X = v2t(x);
 R1 = X(1:2,1:2);
 c1 = R1(1,1); s1 = R1(2,1);

 t = x(1:2);
 theta = x(3);
 delta = l - t;
 % err  = atan2(y,x) - goc_cua_robot - goc_do_duoc
 deltaang = atan2(delta(2),delta(1)); %atan2(y,x)
 temp = deltaang  -theta; % atan2(y,x) - gox_cua_robot 
 temp = atan2(sin(temp),cos(temp)); %(-pi,pi)
 e = temp - z;
 e = atan2(sin(e),cos(e)); %(-pi,pi)
 
 A= 1/(delta'*delta) *[delta(2),-delta(1)]; % Jacobian theo vi tri robot
 Ath = -1;
 A = [A, Ath];
 
 B = 1/(delta'*delta) *[-delta(2),delta(1)]; %Jacobian theo vi tri landmark
 B = [B];

end
