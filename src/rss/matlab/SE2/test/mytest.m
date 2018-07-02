%% Test parseKITTI
clear; clc;
% datapath = 'Data/2D/KITTI00/odometry_0115/';
datapath = 'Data/2D/KITTI00/test/';
MSP = parseKITTI(datapath, 2);
% codesign_plot(MSP.xs, MSP.ys, MSP.g_init, 'Base graph');

% Load hessian of base graph computed by g2o
true_hessian = spconvert(dlmread('Data/2D/KITTI00/test/debug.txt','',6,0));

% base_edges = [MSP.g2o_base_edges; MSP.g2o_cand_edges];
base_edges = [MSP.g2o_base_edges];

%% Test hessian computation (Old code)
% 
% % Load hessian of base graph computed by g2o
% true_hessian = load('Data/2D/KITTI00/test/base_hessian2d.txt');
% true_hessian = spconvert(true_hessian);
% 
% % Compute hessian in matlab!
% dimension = 3; % SE2
% information = eye(dimension); % hardcoded information matrix for now
% fix_vertex = MSP.N; % fix the last vertex for convenience
% 
% % base_edges = [MSP.g2o_base_edges; MSP.g2o_cand_edges];
% base_edges = [MSP.g2o_base_edges];
% assert(size(base_edges, 1) >= MSP.N-1) % When computing information matrix, we must have a connected base graph!
% 
% hessian = sparse(dimension * (MSP.N-1), dimension * (MSP.N-1));
% for i =1:size(base_edges, 1)
%    idi = MSP.vid_to_id(base_edges(i,1)); % convert from g2o index to matlab index
%    idj = MSP.vid_to_id(base_edges(i,2));
%    xi = [MSP.id_to_x(idi); MSP.id_to_y(idi); MSP.id_to_theta(idi)];
%    xj = [MSP.id_to_x(idj); MSP.id_to_y(idj); MSP.id_to_theta(idj)];
%    zij = [base_edges(i, 3); base_edges(i, 4); base_edges(i, 5)]; % observed translation and rotation
%    
%    [Hii, Hij, Hjj] = block_hessian_SE2(idi, idj, xi, xj, zij, information);
%    
%    % add block to corresponding location in the full hessian
%    % because we fix the last vertex, we can use idi and idj directly
%    i_start = (idi-1) * dimension + 1;
%    i_end = idi * dimension;
%    j_start = (idj-1) * dimension + 1;
%    j_end = idj * dimension;
%    
%    % for edges with fixed vertex, only add diagonal block of the other
%    % vertex
%    if idi == fix_vertex
%        hessian(j_start:j_end, j_start:j_end) = hessian(j_start:j_end, j_start:j_end) + Hjj;
%    elseif idj == fix_vertex
%        hessian(i_start:i_end, i_start:i_end) = hessian(i_start:i_end, i_start:i_end) + Hii;
%    else   
%        % diagonal blocks
%        hessian(i_start:i_end, i_start:i_end) = hessian(i_start:i_end, i_start:i_end) + Hii;
%        hessian(j_start:j_end, j_start:j_end) = hessian(j_start:j_end, j_start:j_end) + Hjj;
%        % non-diagonal blocks
%        hessian(i_start:i_end, j_start:j_end) = hessian(i_start:i_end, j_start:j_end) + Hij;
%        hessian(j_start:j_end, i_start:i_end) = hessian(j_start:j_end, i_start:i_end) + Hij';
%    end
% end

%%
base_edges_prob = ones(size(base_edges,1), 1);
hessian = hessian_SE2(MSP, base_edges, base_edges_prob, 1);
hessian = full(hessian)
true_hessian = full(true_hessian)
% det(hessian)










