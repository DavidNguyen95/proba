% Computes the total error of the graph
function Fx = compute_global_error(g)

Fx = 0;

% Loop over all edges
for eid = 1:length(g.edges)
  edge = g.edges(eid);

  % pose-pose constraint
  if (strcmp(edge.type, 'P') ~= 0)

    x1 = v2t(g.x(edge.fromIdx:edge.fromIdx+2));  % the first robot pose
    x2 = v2t(g.x(edge.toIdx:edge.toIdx+2));      % the second robot pose

    %compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    Z = v2t(edge.measurement);
    info_matrix = edge.information;
    
    e = Z\(x1\x2);
    e = t2v(e);
    
    eij = e'*info_matrix*e;
    Fx = Fx + eij;

  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') ~= 0)
    x = g.x(edge.fromIdx:edge.fromIdx+2);  % the robot pose
    l = g.x(edge.toIdx:edge.toIdx+1);      % the landmark

    % compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    X = v2t(x);
    L = v2t([l;0]);
    Z = v2t([edge.measurement;0]);
    info_matrix = edge.information;
    
    e = Z\(X\L);
    e = t2v(e);
    e = e(1:2);
    eij = e'*info_matrix*e;
    Fx = Fx + eij;

  end

end
