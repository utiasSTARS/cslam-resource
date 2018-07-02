% Expected FIM metric
% score = logdet(E(FIM)) >= E(logdet(FIM))
function score = EFIM_metric(MSP, g)
    fix_vertex = 1;
    score = mylogdet(hessian_SE2(MSP, g.g2o_edges, g.g2o_edges_prob, fix_vertex));
end