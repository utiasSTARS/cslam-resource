%% Load dataset
clear; clc;

% Parameters
n_agents = 5;
K_comms = [1 5 10:10:150];
normalize = true;

datapath = 'data/KITTI_MSP';
savepath = 'data/KITTI_result';

load(datapath);

g_best.edges = [MSP.g_init.edges; MSP.candidates];
g_best.g2o_edges = [MSP.g2o_base_edges; MSP.g2o_cand_edges];
g_best.g2o_edges_prob = [ones(size(MSP.g2o_base_edges,1), 1); MSP.candidates(:, 5)];

%% Expected Number of True Loop Closures (NLC)
vertex_greedy_sol = [];
edge_greedy_sol = [];
random_sol = [];
for i = 1:size(K_comms, 2)
   K_comm = K_comms(i);
   fprintf('Computing solutions for K_comm = %d\n', K_comm);
   
   % vertex greedy policy
   fprintf('=== Vertex Greedy ===\n');
   g = vertex_greedy_maxprob(MSP, K_comm);
   validate_solution(MSP, g);
   vertex_greedy_sol = [vertex_greedy_sol g];
   
   % edge greedy policy
   fprintf('=== Edge Greedy ===\n');
   g = edge_greedy_maxprob(MSP, K_comm);
   validate_solution(MSP, g);
   edge_greedy_sol = [edge_greedy_sol g];
   
   % random
   fprintf('=== Random ===\n');
   g = vertex_random(MSP, K_comm);
   validate_solution(MSP, g);
   random_sol = [random_sol g];
end

% Evaluation
nlc_result.K_comms = K_comms;
nlc_result.best_val = maxprob_metric(MSP, g_best);
nlc_result.vgreedy_vals = [];
nlc_result.egreedy_vals = [];
nlc_result.cvx_vals = [];
nlc_result.rand_vals = [];
for i = 1:size(K_comms, 2)
    g = vertex_greedy_sol(i);
    nlc_result.vgreedy_vals = [nlc_result.vgreedy_vals maxprob_metric(MSP, g)];
    g = edge_greedy_sol(i);
    nlc_result.egreedy_vals = [nlc_result.egreedy_vals maxprob_metric(MSP, g)];
    g = random_sol(i);
    nlc_result.rand_vals = [nlc_result.rand_vals maxprob_metric(MSP, g)];
    
    % Convex relaxation upper bound
    [~, ~, val] = maxprob_cvx2(MSP, K_comms(i));
    nlc_result.cvx_vals = [nlc_result.cvx_vals val];
end


%% Tree-connectivity (WST)
edge_greedy_sol = [];
maxprob_sol = [];
random_sol = [];
for i = 1:size(K_comms, 2)
   K_comm = K_comms(i);
   fprintf('Computing solutions for K_comm = %d\n', K_comm);
   
   % edge greedy policy
   fprintf('=== Edge Greedy ===\n');
   g = wst_edge_greedy_alternative(MSP, K_comm, true);
   validate_solution(MSP, g);
   edge_greedy_sol = [edge_greedy_sol g];
   
   
   % rand policy
   fprintf('=== Random ===\n');
   g = vertex_random(MSP, K_comm);
   validate_solution(MSP, g);
   random_sol = [random_sol g];
  
end


% Evaluation
wst_result.K_comms = K_comms;
wst_result.best_val = EWST_metric(MSP, g_best);
wst_result.vgreedy_vals = wst_uniform_vertex_greedy(MSP, K_comms)';
wst_result.egreedy_vals = [];
wst_result.maxprob_vals = [];
wst_result.cvx_vals = [];
wst_result.rand_vals = [];

for i = 1:size(K_comms, 2)
    g = edge_greedy_sol(i);
    wst_result.egreedy_vals = [wst_result.egreedy_vals EWST_metric(MSP, g)];
    g = random_sol(i);
    wst_result.rand_vals = [wst_result.rand_vals EWST_metric(MSP, g)];
    % Convex relaxation upper bound
    [~, ~, val] = wst_cvx2(MSP, K_comms(i));
    wst_result.cvx_vals = [wst_result.cvx_vals val];
end

%% D-optimality Criterion (FIM)

vertex_greedy_sol = [];
edge_greedy_sol = [];
maxprob_sol = [];
random_sol = [];
for i = 1:size(K_comms, 2)
   K_comm = K_comms(i);
   fprintf('Computing solutions for K_comm = %d\n', K_comm);
   
   % vertex greedy policy
   g = vertex_greedy_SE2(MSP, K_comm, true);
   validate_solution(MSP, g);
   vertex_greedy_sol = [vertex_greedy_sol g];
   
   % edge greedy policy
   g = edge_greedy_SE2(MSP, K_comm, true);
   validate_solution(MSP, g);
   edge_greedy_sol = [edge_greedy_sol g];
   
   % rand policy
   fprintf('=== Random ===\n');
   g = vertex_random(MSP, K_comm);
   validate_solution(MSP, g);
   random_sol = [random_sol g];
end


% Evaluation
fim_result.K_comms = K_comms;
fim_result.best_val = EFIM_metric(MSP, g_best);
fim_result.vgreedy_vals = [];
fim_result.egreedy_vals = [];
fim_result.rand_vals = [];

for i = 1:size(K_comms, 2)
    g = vertex_greedy_sol(i);
    fim_result.vgreedy_vals = [fim_result.vgreedy_vals EFIM_metric(MSP, g)];
    g = edge_greedy_sol(i);
    fim_result.egreedy_vals = [fim_result.egreedy_vals EFIM_metric(MSP, g)];
    g = random_sol(i);
    fim_result.rand_vals = [fim_result.rand_vals EFIM_metric(MSP, g)];
end


%% Save data
save(savepath, 'nlc_result', 'wst_result', 'fim_result');




