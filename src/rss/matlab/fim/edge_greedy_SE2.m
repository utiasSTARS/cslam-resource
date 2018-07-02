function g_greedy = edge_greedy_SE2(MSP, K_comm, use_incremental)
fprintf('=== Edge Greedy SE2 ===\n');
if nargin < 3
    use_incremental = true;
end
fix_vertex=1;
n = sum(MSP.nV);
g_greedy = MSP.g_init;
VC = [];
g_greedy.g2o_edges = MSP.g2o_base_edges;
g_greedy.g2o_edges_prob = ones(size(g_greedy.g2o_edges,1), 1);
selected_candidates = [];
curr_cost = 0;

candidates = MSP.candidates;
candidates_vis = MSP.candidates_vis;
g2o_cand_edges = MSP.g2o_cand_edges;

% Make weight constraints for the vertex cover
w_vertex_cover = sparse(n, 1);
w_vertex_cover(MSP.Vlist(:,1)) = MSP.Vlist(:,2);
w_vertex_cover = full(w_vertex_cover);

%% Comm vec setup (for efficient experiments)
comm_is_vec = false;
K_comm_vec = K_comm;
if length(K_comm) > 1
    g_greedy = setup_outputs(g_greedy, K_comm_vec);
    comm_is_vec = true;
end

for comm_index=1:length(K_comm_vec)
    K_comm = K_comm_vec(comm_index);

    while true
        delta = K_comm - curr_cost;
        delta = floor(delta / max(MSP.Vlist(:,2)));
        if delta == 0
            break
        end

        % greedily pick delta edges
        for z = 1:delta
            best.idx = -1;
            best.gain = 0;

            baseH = hessian_SE2(MSP, g_greedy.g2o_edges, g_greedy.g2o_edges_prob, fix_vertex);
            [R,~,S] = chol(baseH);
            g_greedy.val = mylogdet(baseH);

            for j=1:size(candidates, 1)
                g_tmp = g_greedy;
                p = candidates(j,5);
                assert( p >= 0 && p <= 1);
                new_g2o_edges = [g2o_cand_edges(j,:)];
                new_g2o_edges_prob = [p];
                g_tmp.edges = [g_tmp.edges; candidates(j,:)];
                g_tmp.edges_vis = [g_tmp.edges_vis; candidates_vis(j,:)];
                g_tmp.g2o_edges = [g_tmp.g2o_edges; new_g2o_edges];
                g_tmp.g2o_edges_prob = [g_tmp.g2o_edges_prob; new_g2o_edges_prob];

                if ~use_incremental
                    % naively computing difference
                    g_tmp.val = mylogdet(hessian_SE2(MSP, g_tmp.g2o_edges, g_tmp.g2o_edges_prob, fix_vertex));
                    tmp_gain = (g_tmp.val - g_greedy.val);
                else
                    % incremental
                    tmp_gain = inc_gain_SE2(MSP, R, S, new_g2o_edges, new_g2o_edges_prob, fix_vertex);
                    g_tmp.val = g_tmp.val + tmp_gain;
                end

                if tmp_gain >= best.gain
                   best.gain = tmp_gain;
                   best.idx = j;
                   best.G = g_tmp;
                end
            end

            if best.idx == -1
                assert(size(candidates,1) == 0);
                break; % Out of candidates!
            end
            
            if (best.gain == 0)
                fprintf('Warning. Zero gain.\n');
            end
            g_greedy = best.G;
            selected_candidates = [selected_candidates; candidates(best.idx,:)];
            candidates(best.idx,:) = [];
            candidates_vis(best.idx,:) = [];
            g2o_cand_edges(best.idx,:) = [];
        end

        % recompute vertex cover
        VC = grMinVerCoverGurobi(selected_candidates(:,1:2), w_vertex_cover);
        curr_cost = sum(MSP.Vlist(ismember(MSP.Vlist(:,1), VC),2));
        assert(curr_cost <= K_comm);  
        
        if best.idx == -1
            assert(size(candidates,1) == 0);
            break; % Out of candidates!
        end
    end

    % add comm free edges!
    inds_removed = [];
    for i=1:size(candidates,1)
        edge = candidates(i,:);
        u = candidates(i,1);
        v = candidates(i,2);
        p = candidates(i,5);
        assert( p >= 0 && p <= 1);
        if ismember(u, VC) || ismember(v, VC)
            assert(~ismember(edge, g_greedy.edges, 'rows'));
            g_greedy.edges = [g_greedy.edges; candidates(i,:)];
            g_greedy.edges_vis = [g_greedy.edges_vis; candidates_vis(i,:)];
            g_greedy.g2o_edges = [g_greedy.g2o_edges; g2o_cand_edges(i,:)];
            g_greedy.g2o_edges_prob = [g_greedy.g2o_edges_prob; p];
            inds_removed = [inds_removed; i];
        end
    end
    if comm_is_vec
        candidates(inds_removed,:) = [];
        candidates_vis(inds_removed,:) = [];
    end
    g_greedy.val = mylogdet(hessian_SE2(MSP, g_greedy.g2o_edges, g_greedy.g2o_edges_prob, fix_vertex));
    g_greedy.VC = VC;
    
    if comm_is_vec
        g_greedy = set_metrics(g_greedy, MSP, comm_index);
    end

end
