% see how the comm cost evolves as the graph changes

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;
clc;
close all;
addpath(genpath('/home/kasra/git/scanexchange17'))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% create an empty graph
nL = 30;
nR = 30;
g = BipartiteGraphEmpty(nL,nR,randi(3,nL+nR,1).*ones(nL+nR,1));


g.E = [];
for i=1:nL
    for j=1:nR
        if rand > 0.86
            g.E = [g.E ; i nL+j];
        end
    end
end

degs = zeros(nL+nR,1);

edge_weights = 5*rand(size(g.E,1),1);

for i=1:nL
    degs(i,1)    = length(find(g.E(:,1) == i));
    %degs(i,1)    = sum(edge_weights(find(g.E(:,1) == i)));
end
for i=1:nR
    degs(i+nL,1) = length(find(g.E(:,2) == i+nL));
    %degs(i+nL,1)    = sum(edge_weights(find(g.E(:,2) == i+nL)));
end

orig_weights = 30*rand(size(degs));

for omega=0.1:0.1:5
    alpha1 = 1;
    weights= omega*[alpha1*ones(nL,1);alpha1*ones(nR,1)] .* degs + orig_weights;

    vc = grMinVerCover(g.E,weights);
    
    h = grPlot([g.V weights],g.E,'g','%d','',1,vc);
    pause
end
%
% data = [];
% for i=1:nL*nR
%     disp(sprintf("num of edges left %d\n",length(g.E)))
%     cost_with_edge = length(grMinVerCover(g.E,g.Wv));
%     cost_without_edge = length(grMinVerCover(g.E(1:end-1,:),g.Wv));
%     assert(cost_with_edge - cost_without_edge <= 1);
%     assert(cost_with_edge - cost_without_edge >= 0);
%     data = [data ; cost_with_edge - cost_without_edge];
%     g.E = g.E(1:end-1,:);
% end
%
% stairs(1:nL*nR,data);