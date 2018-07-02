function score = EWST_metric(MSP, g)
assert(size(g.edges,2) == 6); % check the passed in graph follows the latest convention

n = MSP.N;
all_edges = [g.edges; MSP.g_init.fake_edges];
m = size(all_edges,1);

A = sparse(n,m);
Wp = sparse(m,m);
Wt = sparse(m,m);

for i=1:m
   u = all_edges(i,1);
   v = all_edges(i,2);
   wp = all_edges(i,3);
   wt = all_edges(i,4);
   p = all_edges(i,5);
   A(u,i) = 1;
   A(v,i) = -1;
   Wp(i,i) = wp * p;
   Wt(i,i) = wt * p;
end

Lp = A * Wp * A';
Lt = A * Wt * A';

score = 2 * mylogdet(Lp(2:end,2:end)) + mylogdet(Lt(2:end,2:end));
end