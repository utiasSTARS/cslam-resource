% nL & nR are the number of vertices
% Wv is the vertex weights
function graph = BipartiteGraphEmpty(nL,nR,Wv)

graph.nL = nL;
graph.nR = nR;

% vertices
graph.L = [];
graph.R = [];

for i=1:graph.nL 
    graph.L = [graph.L; 1  20*i];  
end

for i=1:graph.nR 
    graph.R = [graph.R; 200 20*i]; 
end

graph.V = [graph.L;graph.R];
graph.Wv = Wv;