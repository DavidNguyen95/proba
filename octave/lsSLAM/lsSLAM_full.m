more off;
%clear all;
close all;
addpath '../'
addpath '../../'
addpath '../tools'

g = read_graph_octave('datasets/data_bearing_only.g2o');
%g = read_graph_octave('datasets/dataset_point.g2o');

plot_graph(g, 0);

% printf('Initial error %f\n', compute_global_error(g));

% the number of iterations
numIterations = 1000;

% maximum allowed dx
EPSILON = 10^-4;

% Error
err = 0;
lambda = 1;
bearing  = true;
% carry out the iterations
 for i = 1:numIterations
   printf('Performing iteration %d\n', i);
 
   [dx,lambda] = linearize_and_solve(g,lambda, bearing);
 
%   % TODO: apply the solution to the state vector g.x
   g.x = g.x + dx;
%   % plot the current state of the graph
   plot_graph(g, i);
% tinh gia tri c?a error sau moi lan toi uu
   err = compute_global_error(g);
% 
%   % Print current error
   printf('Current error %f and lambda %f\n', err, lambda);
% 
%   
% 
 end
% 
 printf('Final error %f\n', err);
