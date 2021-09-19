% performs one iteration of the Gauss-Newton algorithm
% each constraint is linearized and added to the Hessian

function [dx, lambda] = linearize_and_solve(g, lambda, bearing)

nnz = nnz_of_graph(g);
% allocate the sparse H and the vector b
H = spalloc(length(g.x), length(g.x), nnz);
b = zeros(length(g.x), 1);

needToAddPrior = true;

% compute the addend term to H and b for each of our constraints
disp('linearize and build system');
for eid = 1:length(g.edges)
  edge = g.edges(eid);

  % pose-pose constraint
  if (strcmp(edge.type, 'P') ~= 0)
    % edge.fromIdx and edge.toIdx describe the location of
    % the first element of the pose in the state vector
    % You should use also this index when updating the elements
    % of the H matrix and the vector b.
    % edge.measurement is the measurement
    % edge.information is the information matrix
    x1 = g.x(edge.fromIdx:edge.fromIdx+2);  % the first robot pose
    x2 = g.x(edge.toIdx:edge.toIdx+2);      % the second robot pose

    % Computing the error and the Jacobians
    % e the error vector
    % A Jacobian wrt x1
    % B Jacobian wrt x2
    info_matrix = edge.information;
    [e, A, B] = linearize_pose_pose_constraint(x1, x2, edge.measurement);


    % TODO: compute and add the term to H and b
    hii = A' * info_matrix * A;
    hij = A' * info_matrix * B;
    hji = B' * info_matrix * A;
    hjj = B' * info_matrix * B;
    
    bi = e'*info_matrix*A;
    bj = e'*info_matrix*B;
    
    i = edge.fromIdx;
    j = edge.toIdx;
    
    H(i:i+2,i:i+2) = H(i:i+2,i:i+2) + hii;
    H(i:i+2,j:j+2) = H(i:i+2,j:j+2) + hij;
    H(j:j+2,i:i+2) = H(j:j+2,i:i+2) + hji;
    H(j:j+2,j:j+2) = H(j:j+2,j:j+2) + hjj;
    
    b(i:i+2) = b(i:i+2) + bi';
    b(j:j+2) = b(j:j+2) + bj';
    


    if (needToAddPrior)
      % TODO: add the prior for one pose of this edge
      % This fixes one node to remain at its current location
      H(1:3,1:3) = H(1:3,1:3) + eye(3,3);
      needToAddPrior = false;
    end

  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') ~= 0)
    % edge.fromIdx and edge.toIdx describe the location of
    % the first element of the pose and the landmark in the state vector
    % You should use also this index when updating the elements
    % of the H matrix and the vector b.
    % edge.measurement is the measurement
    % edge.information is the information matrix
    x1 = g.x(edge.fromIdx:edge.fromIdx+2);  % the robot pose
    x2 = g.x(edge.toIdx:edge.toIdx+1);      % the landmark

    % Computing the error and the Jacobians
    % e the error vector
    % A Jacobian wrt x1
    % B Jacobian wrt x2
    


    % TODO: compute and add the term to H and b
    info_matrix = edge.information;
    [e, A, B] = linearize_pose_landmark_constraint(x1, x2, edge.measurement);


    % TODO: compute and add the term to H and b
    hii = A' * info_matrix * A;
    hij = A' * info_matrix * B;
    hji = B' * info_matrix * A;
    hjj = B' * info_matrix * B;
    
    bi = e'*info_matrix*A;
    bj = e'*info_matrix*B;
    
    i = edge.fromIdx;
    j = edge.toIdx;
    
    H(i:i+2,i:i+2) = H(i:i+2,i:i+2) + hii;
    H(i:i+2,j:j+1) = H(i:i+2,j:j+1) + hij;
    H(j:j+1,i:i+2) = H(j:j+1,i:i+2) + hji;
    H(j:j+1,j:j+1) = H(j:j+1,j:j+1) + hjj;
    
    b(i:i+2) = b(i:i+2) + bi';
    b(j:j+1) = b(j:j+1) + bj';
    
   
  elseif (strcmp(edge.type, 'LB') ~= 0)
    % edge.fromIdx and edge.toIdx describe the location of
    % the first element of the pose and the landmark in the state vector
    % You should use also this index when updating the elements
    % of the H matrix and the vector b.
    % edge.measurement is the measurement
    % edge.information is the information matrix
    x1 = g.x(edge.fromIdx:edge.fromIdx+2);  % the robot pose
    x2 = g.x(edge.toIdx:edge.toIdx+1);      % the landmark

    % Computing the error and the Jacobians
    % e the error vector
    % A Jacobian wrt x1
    % B Jacobian wrt x2
    


    % TODO: compute and add the term to H and b
    info_matrix = edge.information;
    [e, A, B] = linearize_pose_landmark_bearing_constraint(x1, x2, edge.measurement);


    % TODO: compute and add the term to H and b
    hii = A' * info_matrix * A;
    hij = A' * info_matrix * B;
    hji = B' * info_matrix * A;
    hjj = B' * info_matrix * B;
    
    bi = e'*info_matrix*A;
    bj = e'*info_matrix*B;
    
    i = edge.fromIdx;
    j = edge.toIdx;
    
    H(i:i+2,i:i+2) = H(i:i+2,i:i+2) + hii;
    H(i:i+2,j:j+1) = H(i:i+2,j:j+1) + hij;
    H(j:j+1,i:i+2) = H(j:j+1,i:i+2) + hji;
    H(j:j+1,j:j+1) = H(j:j+1,j:j+1) + hjj;
    
    b(i:i+2) = b(i:i+2) + bi';
    b(j:j+1) = b(j:j+1) + bj';
    
  end

end

disp('solving system');

% TODO: solve the linear system, whereas the solution should be stored in dx
% Remember to use the backslash operator instead of inverting H
if (bearing)
    H = H + lambda*eye(length(g.x));
    err0 = compute_global_error(g); 
    dx = -H\b;
    temp = g;
    temp.x = temp.x + dx;
    err = compute_global_error(temp);
    if (err > err0) && (lambda <= 128)
      dx = 0;
      lambda = lambda*2;
      printf('toan --------------------------------- tang lambda: %f\n', lambda);
    else
      lambda = max(1/256,lambda/2);
      printf('toan ----------------------------------giam lambda: %f\n', lambda);
    endif
else
   dx = -H\b;
endif

end
