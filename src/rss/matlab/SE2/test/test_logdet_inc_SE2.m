%% Test logdet_inc with logdet(I)

%% Load KITTI data
clear; clc;
format long;
datapath = 'Data/2D/KITTI00/odometry_0115/';
MSP = parseKITTI(datapath, 2);

% codesign_plot(MSP.xs, MSP.ys, MSP.g_init, 'Base graph');
base_edges = MSP.g2o_base_edges;
cand_edges = MSP.g2o_cand_edges;
%% Test logdet_inc
fix_vertex = 1;
tol = 1e-4;

% hardcoded information matrix for now
information = eye(3);

% Compute cholesky of base hessian
base_edges_prob = ones(size(base_edges,1), 1);
baseH = hessian_SE2(MSP, base_edges, base_edges_prob, fix_vertex);
basel = log(det(baseH));
[R,~,S] = chol(sparse(baseH));

M = size(cand_edges, 1);
for l = 1:100
   fprintf('=== Iteration %d ===\n', l); 
   
   tic;
   % randomly sample edges from candidates
   set_size = randi([2,50]);
   set = randsample(1:M, set_size ,false);
   
   
   new_edges = cand_edges(set, :);
   new_edges_prob = rand(set_size,1);

   inc = inc_gain_SE2(MSP, R, S, new_edges, new_edges_prob, fix_vertex);
   fprintf('logdet_inc elapse time: %f\n', toc);
   
   % compute inc in naive way
   tic;
   
   newl = log(det(hessian_SE2(MSP, [base_edges; new_edges], [base_edges_prob; new_edges_prob], fix_vertex)));
   fprintf('naive diff elapse time: %f\n', toc);
   
   
   diff = abs((newl - basel) - inc);
   fprintf('Diff:%f\n', diff);
   assert(diff < tol);
end

