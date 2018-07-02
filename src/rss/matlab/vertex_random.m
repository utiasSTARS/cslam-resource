% Vlist_rand is a fixed random permutation of Vlist
function g_rand = vertex_random(MSP, K_comm)
candidates = MSP.candidates;
candidates_vis = MSP.candidates_vis;
Vlist = MSP.Vlist;
g_rand = MSP.g_init;
g_rand.g2o_edges = MSP.g2o_base_edges;
g_rand.g2o_edges_prob = ones(size(g_rand.g2o_edges,1), 1);
rand_idxs = randsample(size(Vlist,1), K_comm);
g_rand.VC = Vlist(rand_idxs,1);


for i = 1:size(candidates,1)
    u = candidates(i,1);
    v = candidates(i,2);
    if ismember(u, g_rand.VC) || ismember(v, g_rand.VC)
        g_rand.edges = [g_rand.edges; candidates(i,:)];
        g_rand.edges_vis = [g_rand.edges_vis; candidates_vis(i,:)]; 
        g_rand.g2o_edges = [g_rand.g2o_edges; MSP.g2o_cand_edges(i,:)];
        g_rand.g2o_edges_prob = [g_rand.g2o_edges_prob; candidates(i,5)];
    end
end

end