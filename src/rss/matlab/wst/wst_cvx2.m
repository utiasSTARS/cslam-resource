% Convex Relaxation
% The value of the obtained solution is an upperbound of the optimal
% solution.
% This implementation creates a variable for each candidate vertex 
% (i.e. connected to at least one candidate edge)
% Output: 
% xopt - decision variables over vertices
% yopt - decision variables over edges
% optval - optimal value

function [xopt,yopt,optval] = wst_cvx2(MSP, K_comm)
fprintf('=== WST_CVX2 ===\n')

addpath(genpath('~/YALMIP'));
addpath(genpath('~/SDPT3'));

% number of candidate vertices
n = size(MSP.Vlist,1);
% number of candidate edges
m = size(MSP.candidates,1);

% weight of each candidate vertex
w = ones(n,1);

% creating mappings from actual idx to variable idx
ver2idx = containers.Map('KeyType','int32','ValueType','int32');
idx2ver = containers.Map('KeyType','int32','ValueType','int32');

for i = 1:n
   vertex = MSP.Vlist(i,1);
   w(i) = MSP.Vlist(i,2);
   ver2idx(vertex) = i;
   idx2ver(i) = vertex;
end

% vertex vars
x = sdpvar(n, 1);
% edge vars
y = sdpvar(m, 1);

% base laplacians
Lp_base = MSP.g_init.Lp;
Lt_base = MSP.g_init.Lt;

% Incidence matrix for candidate edges
A = sparse(MSP.N, m);
for i = 1:m
   u = MSP.candidates(i,1);
   v = MSP.candidates(i,2);
   A(u, i) = 1;
   A(v, i) = -1;
end

% Weight matrix
Wp = sparse(m,m);
Wt = sparse(m,m);
for i = 1:m
   Wp(i,i) = MSP.candidates(i,3) * MSP.candidates(i,5); % scale by probability
   Wt(i,i) = MSP.candidates(i,4) * MSP.candidates(i,5);
end

Lp = Lp_base(2:end, 2:end) + A(2:end,:) * Wp * spdiags(y(:),0,m,m) *A(2:end,:)';
Lt = Lt_base(2:end, 2:end) + A(2:end,:) * Wt * spdiags(y(:),0,m,m) *A(2:end,:)';

objective = -2*logdet(Lp)-logdet(Lt);
options = sdpsettings('solver','sdpt3');
constraints = [1>= x >=0, 1>= y >=0, w'*x<=K_comm];
% add constraint imposed by each edge
for i=1:m
   u = MSP.candidates(i,1);
   v = MSP.candidates(i,2);
   uidx = ver2idx(u);
   vidx = ver2idx(v);
   constraints = [constraints, x(uidx)+x(vidx) >= y(i)];
end

% Solve!!
sol = optimize(constraints, objective, options);

if sol.problem ~= 0
 sol.info
 yalmiperror(sol.problem)
end

xopt = value(x);
yopt = value(y);
Lp_opt = Lp_base(2:end, 2:end) + A(2:end,:) * Wp * spdiags(yopt(:),0,m,m) *A(2:end,:)';
Lt_opt = Lt_base(2:end, 2:end) + A(2:end,:) * Wt * spdiags(yopt(:),0,m,m) *A(2:end,:)';
optval = 2*logdet(Lp_opt) + logdet(Lt_opt); 
% optval = -value(objective);

end
