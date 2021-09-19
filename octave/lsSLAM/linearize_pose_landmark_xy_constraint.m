% Compute the error of a pose-landmark constraint
% x 3x1 vector (x,y,theta) of the robot pose
% l 2x1 vector (x,y) of the landmark
% z 2x1 vector (x,y) of the measurement, the position of the landmark in
%   the coordinate frame of the robot given by the vector x
%
% Output
% e 2x1 error of the constraint
% A 2x3 Jacobian wrt x
% B 2x2 Jacobian wrt l
function [e, A, B] = linearize_pose_landmark_constraint(x, l, z)

  % TODO compute the error and the Jacobians of the error
 X = v2t(x);
 R1 = X(1:2,1:2);
 c1 = R1(1,1); s1 = R1(2,1);
 R1_t = [[-s1 -c1];[c1 -s1]];
 
 t = x(1:2);
 e = R1'*(l -t) - z;
 A = [-R1' , R1_t'*(l-t)]; %Jacobian theo vi tri robot

 B = R1'; %Jacobian cua landmark

end
