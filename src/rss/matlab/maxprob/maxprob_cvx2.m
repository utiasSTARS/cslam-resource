function [xopt, yopt, optval] = maxprob_cvx2(MSP, K_comm)

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

% Objective
ps = MSP.candidates(:,5);
objective = -ps'*y;

% options = sdpsettings('solver','sdpt3');
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
sol = optimize(constraints, objective);

if sol.problem ~= 0
 sol.info
 yalmiperror(sol.problem)
end

xopt = value(x);
yopt = value(y);
optval = -value(objective);


end