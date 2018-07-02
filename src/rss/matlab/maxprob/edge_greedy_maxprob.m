function g_greedy = edge_greedy_maxprob(MSP, K_comm)

fprintf('=== Edge Greedy MaxProb ===\n');

n = MSP.N;
candidates = MSP.candidates;
candidates_vis = MSP.candidates_vis;
g2o_cand_edges = MSP.g2o_cand_edges;
g_greedy = MSP.g_init;

selected_candidates = [];
VC = [];
curr_cost = 0;

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
            break;
        end

        % greedily pick delta edges
        for j = 1:delta
            [~, idx] = max(candidates(:, 5));
            selected_candidates = [selected_candidates; candidates(idx,:)];
            g_greedy.edges = [g_greedy.edges; candidates(idx,:)];
            g_greedy.edges_vis = [g_greedy.edges_vis; candidates_vis(idx,:)];
            candidates(idx, :) = [];
            candidates_vis(idx, :) = [];
        end

        % recompute vertex cover
        VC = grMinVerCoverGurobi(selected_candidates(:,1:2), w_vertex_cover);
        curr_cost = sum(MSP.Vlist(ismember(MSP.Vlist(:,1), VC),2));
        assert(curr_cost <= K_comm);  
    end
    

    % add comm free edges!
%     inds_removed = [];
%     for i=1:size(candidates,1)
%         edge = candidates(i,:);
%         u = candidates(i,1);
%         v = candidates(i,2);
%         p = candidates(i,5);
%         assert( p >= 0 && p <= 1);
%         if ismember(u, VC) || ismember(v, VC)
%             assert(~ismember(edge, g_greedy.edges, 'rows'));
%             g_greedy.edges = [g_greedy.edges; candidates(i,:)];
%             g_greedy.edges_vis = [g_greedy.edges_vis; candidates_vis(i,:)];
%             g_greedy.g2o_edges = [g_greedy.g2o_edges; g2o_cand_edges(i,:)];
%             g_greedy.g2o_edges_prob = [g_greedy.g2o_edges_prob; p];
%             inds_removed = [inds_removed; i];
%         end
%     end
%     
%     if comm_is_vec
%         candidates(inds_removed,:) = [];
%         candidates_vis(inds_removed,:) = [];
%         g_greedy = set_metrics(g_greedy, MSP, comm_index);
%     end

    % recompute vertex cover
%     VC = grMinVerCoverGurobi(selected_candidates(:,1:2), w_vertex_cover);
%     curr_cost = sum(MSP.Vlist(ismember(MSP.Vlist(:,1), VC),2));
%     assert(curr_cost <= K_comm);

end

% add comm free edges!
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
    end
end


g_greedy.VC = VC;
g_greedy.val = sum(selected_candidates(:,5));

end
