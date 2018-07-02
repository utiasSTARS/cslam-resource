function gain = inc_gain_wst(MSP, Rp, Sp, Rt, St, new_edges)
candidates = MSP.candidates;
n = size(MSP.g_init.Lp, 1);
At = sparse(n, size(new_edges,1));
Ap = sparse(n, size(new_edges,1));

for i = 1:size(new_edges, 1)
    e = new_edges(i);
    u = candidates(e,1);
    v = candidates(e,2);
    wp = candidates(e,3);
    wt = candidates(e,4);
    p = candidates(e,5);
    At(u, i) = sqrt(p*wt);
    At(v, i) = -sqrt(p*wt);
    Ap(u, i) = sqrt(p*wp);
    Ap(v, i) = -sqrt(p*wp);
end
At = At(2:end,:);
Ap = Ap(2:end,:);

gain = 2*logdet_inc(Rp, Sp, Ap) + logdet_inc(Rt, St, At);

end