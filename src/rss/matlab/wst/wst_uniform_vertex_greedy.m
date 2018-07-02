% Vertex greedy for WST codesign
% Ignore computation constraint
% Vectorized for communication budgets
% Input:
% - MSP: problem instance
% - K_comms: array of communication budgets
% Output:
% - vals: objective value achieved for each combination of constraints (column vector)
% - gs: solutions for each budget (column cell array)

function [vals, gs] = wst_uniform_vertex_greedy(MSP, K_comms)
fprintf('=== Codesign WST Uniform Vertex Greedy ===\n');
n = size(MSP.g_init.Lp,1);
vals = zeros(size(K_comms, 2), 1);
gs = cell(size(K_comms, 2), 1);
Vlist = MSP.Vlist;
candidates = MSP.candidates;
candidates_vis = MSP.candidates_vis;
g2o_cand_edges = MSP.g2o_cand_edges;
K_comm_max = min(max(K_comms), size(Vlist,1));

% register edges with vertices
for i=1:size(Vlist,1)
   v = Vlist(i,1);
   edges{v} = [];
end
for j=1:size(candidates, 1)
    u = candidates(j,1);
    v = candidates(j,2);
    edges{u} = [edges{u}; j];
    edges{v} = [edges{v}; j];
end

% base graph laplacians
[initRt,~,initSt] = chol(MSP.g_init.Lt(2:end,2:end));
[initRp,~,initSp] = chol(MSP.g_init.Lp(2:end,2:end));
initval = 2*mylogdet(MSP.g_init.Lp(2:end,2:end)) + mylogdet(MSP.g_init.Lt(2:end,2:end));

% initialize
cost_curr = 0;
zero_gain = false;
edges_curr = []; % selected edges
VC = [];
Lp_curr = MSP.g_init.Lp; % ungrounded laplacian (position) 
Lt_curr = MSP.g_init.Lt; % ungrounded laplacian (orientation)

while cost_curr < K_comm_max
    % greedy loop
    best.gain = -1;
    best.idx = -1;
    
    % base grounded laplacians for this round
    [Rt,~,St] = chol(Lt_curr(2:end,2:end));
    [Rp,~,Sp] = chol(Lp_curr(2:end,2:end));
    
    % go through candidate vertex
    for i=1:size(Vlist,1)       
        edges_tmp = edges_curr;
        s = Vlist(i,1);
        c = Vlist(i, 2);
        assert(c == 1);
        if c > K_comm_max - cost_curr
           continue; 
        end
        % calculate gain by adding new vertex
        edges_tmp = union(edges_tmp, edges{s});
        new_edges = setdiff(edges_tmp, edges_curr);
        tmp_gain = inc_gain_wst(MSP, Rp, Sp, Rt, St, new_edges);
        if tmp_gain >= best.gain
           best.gain = tmp_gain;
           best.idx = i;
           best.c = c;
           best.edges = edges_tmp;
        end
    end
    
    if best.idx == -1
        assert(0 == 1, 'Error: cannot find best candidate.');
    end
    if (best.gain < 1e-6)
        zero_gain = true;
    end
    
    % update
    cost_curr = cost_curr + best.c;
    VC = [VC; Vlist(best.idx,1)];
    new_edges = setdiff(best.edges, edges_curr);
    for i = 1:size(new_edges,1)
        e = new_edges(i);
        u = candidates(e,1);
        v = candidates(e,2);
        wp = candidates(e,3);
        wt = candidates(e,4);
        p = candidates(e,5);
        a = sparse(n,1);
        a(u) = 1;
        a(v) = -1;
        Lp_curr = Lp_curr + p*wp*(a*a');
        Lt_curr = Lt_curr + p*wt*(a*a');
    end
    edges_curr = best.edges;
    Vlist(best.idx,:) = [];
    
    % check if meet communication budget
    K_comm_idx = find(K_comms == cost_curr);
    assert(size(K_comm_idx,2) <= 1);
    if size(K_comm_idx,2) == 1
        val = inc_gain_wst(MSP, initRp, initSp, initRt, initSt, edges_curr);
        
        % construct solution
        g = MSP.g_init;
        g.edges = [g.edges; candidates(edges_curr, :)];
        g.edges_vis = [g.edges_vis; candidates_vis(edges_curr, :)];
        g.g2o_edges = [g.g2o_edges; g2o_cand_edges(edges_curr,:)];
        g.g2o_edges_prob = [g.g2o_edges_prob; candidates(edges_curr, 5)];
        g.VC = VC;

        % save solution in output matrix
        vals(K_comm_idx, 1) = val;
        gs{K_comm_idx, 1} = g;
        fprintf('B = %d: value = %d\n', K_comms(K_comm_idx), val);
        
        % if already encounter zero gain, don't need to compute rest of communication budget
        if zero_gain 
           fprintf('Zero gain: skip remaining communication budgets. \n');
           for i = K_comm_idx+1:size(K_comms,2)
               vals(i, 1) = val;
               gs{i, 1} = g;
           end
           break;
        end
    end
end
end