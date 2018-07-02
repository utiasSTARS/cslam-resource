% greedily select vertices based on logdet(FIM)
function g_greedy = vertex_greedy_maxprob(MSP, K_comm, uniform)
    fprintf('=== Vertex Greedy MaxProb ===\n');
    if nargin < 3
       uniform = false; 
    end
    g_greedy = MSP.g_init;
    g_greedy.g2o_edges = MSP.g2o_base_edges;
    g_greedy.g2o_edges_prob = ones(size(g_greedy.g2o_edges,1), 1);
    assert(size(MSP.candidates, 2) == 6) % assert each row of candidate is idi, idj, wp, wt, prob, valid
    assert(size(MSP.g2o_cand_edges, 1) == size(MSP.candidates, 1));
    
    Vlist = MSP.Vlist;
    candidates = MSP.candidates;
    candidates_vis = MSP.candidates_vis;
    g2o_cand_edges = MSP.g2o_cand_edges;
    
    VC = []; % keep track of selected vertices
    %% Comm vec setup (for efficient experiments)
    comm_is_vec = false;
    if length(K_comm) > 1
        K_comm_vec = K_comm;
        g_greedy = setup_outputs(g_greedy, K_comm_vec);
        K_comm = K_comm(end);
        comm_is_vec = true;
    end
    curr_comm_cost = 0;
    while K_comm > 0
        % at beginning of each round, go thru each candidate edge and register
        % it with vertices
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
        
        
        % greedy loop
        max.gain = 0;
        max.idx = -1;
        
        g_greedy.val = sum(g_greedy.edges(:,5));
        for i=1:size(Vlist,1) % loop through each candidate vertex
            g_tmp = g_greedy;
            s = Vlist(i,1);
            cost = Vlist(i, 2);
%             assert(cost == 1); 
            if cost > K_comm
               continue; 
            end
            new_edges = edges{s};
            if size(new_edges,1) == 0
               continue; 
            end
            
            new_g2o_edges = [];
            new_g2o_edges_prob = [];
            for k = 1:size(new_edges,1)
                j = new_edges(k);
                p = candidates(j,5);
                assert( p >= 0 && p <= 1);
                new_g2o_edges = [new_g2o_edges; g2o_cand_edges(j,:)];
                new_g2o_edges_prob = [new_g2o_edges_prob; p];
                g_tmp.edges = [g_tmp.edges; candidates(j,:)];
                g_tmp.edges_vis = [g_tmp.edges_vis; candidates_vis(j,:)];
            end
            g_tmp.g2o_edges = [g_tmp.g2o_edges; new_g2o_edges];
            g_tmp.g2o_edges_prob = [g_tmp.g2o_edges_prob; new_g2o_edges_prob];
            
            g_tmp.val = sum(g_tmp.edges(:,5));
            tmp_gain = g_tmp.val - g_greedy.val;
            
            if ~uniform
                tmp_gain = tmp_gain / cost; % gain per unit cost
            end
            if tmp_gain >= max.gain
               max.gain = tmp_gain;
               max.idx = i;
               max.cost = cost;
               max.G = g_tmp;
            end
        end
        
        if max.idx == -1
            break; 
        end
        if (max.gain == 0)
            fprintf('Warning. Zero gain.\n');
        end
        g_greedy = max.G;
        K_comm = K_comm - max.cost;
        VC = [VC; Vlist(max.idx,1)];
        
        % remove selected edges from candidates
        selected_v = Vlist(max.idx);
        selected_edges = sort(edges{selected_v}, 'descend');
        for k = 1:size(selected_edges, 1)
           j = selected_edges(k);
           candidates(j,:) = [];
           candidates_vis(j,:) = [];
           g2o_cand_edges(j,:) = [];
        end
        % remove selected vertex from Vlist
        Vlist(max.idx,:) = [];
        
        curr_comm_cost = curr_comm_cost + max.cost;
        if comm_is_vec 
            comm_index = find(curr_comm_cost == K_comm_vec);
            if ~isempty(comm_index)
                g_greedy = set_metrics(g_greedy, MSP, comm_index);
            end
        end
    end
    g_greedy.VC = VC;
end