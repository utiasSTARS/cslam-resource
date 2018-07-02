function g_best = wst_edge_greedy_alternative(MSP, K_comm, gurobi_flag)

if nargin < 3
    gurobi_flag = true;
end

%% Common Setup
nV = MSP.nV;
candidates = MSP.candidates;
candidates_vis = MSP.candidates_vis;
g2o_cand_edges = MSP.g2o_cand_edges;
Vlist = MSP.Vlist;
g_init = MSP.g_init;

n = sum(nV);
g_best = g_init;
g_best.VC = [];
selected_candidates = [];
curr_cost = 0;
% Make weight constraints for the vertex cover
w_vertex_cover = sparse(n, 1);
w_vertex_cover(Vlist(:,1)) = Vlist(:,2);
w_vertex_cover = full(w_vertex_cover);

%% Comm vec setup (for efficient experiments)
comm_is_vec = false;
K_comm_vec = K_comm;
if length(K_comm) > 1
    g_best = setup_outputs(g_best,K_comm_vec);
    comm_is_vec = true;
end

for comm_index=1:length(K_comm_vec)
    K_comm = K_comm_vec(comm_index);
    while true
       delta = K_comm - curr_cost; % we can always pick this number of new edges without computing vertex cover 
       delta = floor(delta / max(MSP.Vlist(:,2)));
       if delta == 0
           break
       end

       % greedily pick delta edges
       for z = 1:delta
            best_res.idx = -1;
            best_res.gain = 0;
            [Rt,~,St] = chol(g_best.Lt(2:end,2:end));
            g_best.Rt = Rt;
            g_best.St = St;
            [Rp,~,Sp] = chol(g_best.Lp(2:end,2:end));
            g_best.Rp = Rp;
            g_best.Sp = Sp; 

            for j=1:size(candidates, 1)
                g_tmp = g_best;
                edge = candidates(j,:); 
                u = edge(1);
                v = edge(2);
                assert( u~=v , 'u=v');
                wp = edge(3); % edge weight (translation)
                wt = edge(4); % edge weight (rotation)
                p = edge(5);
                a = sparse(n,1);
                a(u) = 1;
                a(v) = -1;

                gain_t = logdet_inc(Rt, St, sqrt(wt*p)*a(2:end));
                gain_p = logdet_inc(Rp, Sp, sqrt(wp*p)*a(2:end));
                gain = 2*gain_p + gain_t;
                g_tmp.val = g_best.val + gain;

                g_tmp.Lp = g_tmp.Lp + p*wp*a*a';
                g_tmp.Lt = g_tmp.Lt + p*wt*a*a'; 
                g_tmp.edges = [g_tmp.edges; edge];
                g_tmp.edges_vis = [g_tmp.edges_vis; candidates_vis(j,:)];

                if gain >= best_res.gain
                    best_res.G = g_tmp;
                    best_res.idx = j;
                    best_res.gain = gain;
                end
            end
            if best_res.idx == -1
                assert(size(candidates,1) == 0);
                break; % Out of candidates!
            end
            g_best = best_res.G;
            selected_candidates = [selected_candidates; candidates(best_res.idx,:)];
            candidates(best_res.idx,:) = [];
            candidates_vis(best_res.idx,:) = [];
       end

       % recompute VC
       VC_prev = g_best.VC;
       if gurobi_flag
           g_best.VC = grMinVerCoverGurobi(selected_candidates(:,1:2), w_vertex_cover);
       else 
           g_best.VC = grMinVerCoverLP(selected_candidates(:,1:2), w_vertex_cover);
       end
       curr_cost_candidate = sum(Vlist(ismember(Vlist(:,1), g_best.VC),2));
       if curr_cost_candidate > K_comm
           % This is a bad 2-opt LP VC result, do 'manual' addition to the VC
           sprintf('\n\nLP VC was worse than naive!\n\n');
           for edge_idx=size(selected_candidates,1)-delta+1:size(selected_candidates,1)
               VC_prev = [VC_prev; selected_candidates(edge_idx,1)];
           end
           g_best.VC = unique(VC_prev);
           curr_cost = sum(Vlist(ismember(Vlist(:,1), g_best.VC), 2));
           assert(curr_cost <= K_comm, 'Manual VC top up failed.');
       else
           curr_cost = curr_cost_candidate;
       end

       % Check for out of candidates situation
       if best_res.idx == -1
           assert(curr_cost <= K_comm);
           break;
       end

    end

    % add comm free edges
    inds_removed = [];
    for i=1:size(candidates,1)
        edge = candidates(i,:);
        u = candidates(i,1);
        v = candidates(i,2);
        wp = candidates(i,3); % edge weight (translation)
        wt = candidates(i,4); % edge weight (rotation)
        p = candidates(i,5);
        a = sparse(n,1);
        a(u) = 1;
        a(v) = -1;
        if ismember(u, g_best.VC) || ismember(v, g_best.VC)
            assert(~ismember(edge, g_best.edges, 'rows'));
            g_best.Lp = g_best.Lp + p*wp*a*a';
            g_best.Lt = g_best.Lt + p*wt*a*a'; 
            g_best.edges = [g_best.edges; candidates(i,:)];
            g_best.edges_vis = [g_best.edges_vis; candidates_vis(i,:)];
            inds_removed = [inds_removed; i];
        end
    end
    if comm_is_vec
        candidates(inds_removed,:) = [];
        candidates_vis(inds_removed,:) = [];
    end
    g_best.val = 2 * mylogdet(g_best.Lp(2:end,2:end)) + mylogdet(g_best.Lt(2:end,2:end));
    if comm_is_vec
        g_best = set_metrics(g_best, MSP, comm_index);
    end

end
                        
end
