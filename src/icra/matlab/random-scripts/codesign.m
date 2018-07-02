%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;
clc;
close all;
addpath(genpath('/home/kasra/git/scanexchange17'))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nL = 500;
nR = 500;
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% k_card = 5;
% k_comm = 3;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vertices
L = [];
R = [];
for i=1:nL L = [L; 1 2*i]; end
for i=1:nR R = [R; 40 2*i]; end
V = [L;R];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% K(n1,n2) edges
E_K = [];
for i=1:nL
    for j=1:nR
        if rand > 0.995
            E_K = [E_K; i nL+j];
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%grPlot(V,E_K,'g','%d','');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for k=k_card:-1:k_comm+1
%     C = VChooseK(1:size(E_K,1), k);
%     for i=1:size(C,1)
%         E = E_K(C(i,:),:)
%         nMC=grMinVerCover(E);
%         if length(nMC) <= k_comm
%             h = grPlot(V,E,'g','%d','');
%             length(nMC)
%             length(E)
%             t = sprintf('Num. Edges: %d          M. Vertex Cover: %d',length(E),length(nMC));
%             title(t)
%             pause
%             close(h)
%         end
%     end
% end

E = E_K;
tic
nMC=grMinVerCover(E)
tt= toc

h = grPlot(V,E,'g','%d','',0,[1,2])
t = sprintf('Num. Edges: %d          M. Vertex Cover: %d',length(E),length(nMC));
title(t)


dis_L = nL - length(unique(E(:,1)));
dis_R = nR - length(unique(E(:,2)));

length(nMC) - min(nL-dis_L , nR-dis_R)