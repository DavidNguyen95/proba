% Compute the error of a pose-pose constraint
% x1 3x1 vector (x,y,theta) of the first robot pose
% x2 3x1 vector (x,y,theta) of the second robot pose
% z 3x1 vector (x,y,theta) of the measurement
%
% You may use the functions v2t() and t2v() to compute
% a Homogeneous matrix out of a (x, y, theta) vector
% for computing the error.
%
% Output
% e 3x1 error of the constraint
% A 3x3 Jacobian wrt x1
% B 3x3 Jacobian wrt x2
function [e, A, B] = linearize_pose_pose_constraint(x1, x2, z)

  % TODO compute the error and the Jacobians of the error
  X1 = v2t(x1);
  X2 = v2t(x2);
  t1 = x1(1:2) ; t2= x2(1:2);
  Z = v2t(z);
  R_1 = X1(1:2,1:2);
  
  R12 = Z(1:2,1:2);
  c1  = R_1(1,1); s1 = R_1(2,1);
  R1_t = [[-s1 -c1];[c1 -s1]];
  e = t2v(Z\(X1\X2));
  
  A_xy = [-R12'*R_1' , R12'*R1_t'*(t2-t1)]; %jacobian cua vi tri i
  B_xy = [R12'*R_1' , [0;0]]; % Jacobian cua vi tri j
  A = [A_xy;[0 0 -1]];
  B = [B_xy;[0 0 1]];

end
