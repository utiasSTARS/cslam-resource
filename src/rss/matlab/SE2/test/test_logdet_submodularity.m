%% Load KITTI data
clear; clc;
datapath = 'Data/2D/KITTI00/odometry_0115/';
MSP = parseKITTI(datapath, 2);
% codesign_plot(MSP.xs, MSP.ys, MSP.g_init, 'Base graph');

% base_edges = [MSP.g2o_base_edges; MSP.g2o_cand_edges];
base_edges = MSP.g2o_base_edges;
cand_edges = MSP.g2o_cand_edges;
%% Check monotone submodularity of logdet(I)
clc;
n = size(cand_edges,1);
profile on;
for l = 1:100
    fprintf('=== Iteration %d ===\n', l)
    % set1 is a superset of set2
    set1_size = randi([2,50]);
    set1 = randsample(1:n, set1_size ,false);
    
    set2_size = randi([1,set1_size]);
    set2 = randsample(set1, set2_size,false);
    
    edges1 = [base_edges; cand_edges(set1, :)];
    edges2 = [base_edges; cand_edges(set2, :)];
   
    val1 = mylogdet(hessian_SE2(MSP, edges1, 1));
    val2 = mylogdet(hessian_SE2(MSP, edges2, 1));
    
    % assert monotone
    assert(val1 >= val2);
    
    % e is a edge outside set1
    while true
        e = randi([1,n]);
        if ismember(e, set1)==false
          break; 
       end
    end
    
    edges1 = [edges1; cand_edges(e,:)];
    edges2 = [edges2; cand_edges(e,:)];
    
    val1_new = mylogdet(hessian_SE2(MSP, edges1, 1));
    val2_new = mylogdet(hessian_SE2(MSP, edges2, 1));
    
    gain1 = val1_new - val1;
    gain2 = val2_new - val2;
    
    % assert submodular
    assert(gain1 <= gain2);
end

profile viewer;


