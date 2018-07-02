%% Toy example graph
clear; 
% Set to true if Gurobi is installed
gurobi_flag = true;

%% Uniform weighted vertices (i.e. measurements all the same size)
W = ones(12,1);

% Graph with 2 components
E = [1 7;
     2 7;
     3 7;
     3 8;
     4 8;
     4 9;
     4 10;
     5 11;
     6 11;
     6 12]; 

[mvc_uniform, cover_uniform] = solve_odep(E, W, gurobi_flag);
fprintf('Cost for each component with uniform weights:');
disp(mvc_uniform);
fprintf('Vertices to exchange for uniform case: \n');
disp(cover_uniform);

%% Random weights
W_rand = rand(size(W))*5 + 1;
[mvc_rand, cover_rand] = solve_odep(E, W_rand, gurobi_flag);
fprintf('Cost for each component with random weights:');
disp(mvc_rand);
fprintf('Vertices to exchange for random case: \n');
disp(cover_rand);