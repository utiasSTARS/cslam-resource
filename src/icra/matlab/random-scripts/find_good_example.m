%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;
clc;
close all;
addpath(genpath('/home/kasra/git/scanexchange17'))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
X = [];
X_rel = [];
x_max = -100;
G = [];

for I=30:40
    for J=I+1:40
        for P=0.8:0.05:0.9
            for R=1:10
                status = sprintf('(I,J,P,R) = (%d,%d,%f,%d)',I,J,P,R);
                disp(status);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                graph.nL = I;
                graph.nR = J;
                % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % k_card = 5;
                % k_comm = 3;
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Vertices
                graph.L = [];
                graph.R = [];
                for i=1:graph.nL graph.L = [graph.L; 1  2*i]; end
                for i=1:graph.nR graph.R = [graph.R; 100 2*i]; end
                graph.V = [graph.L;graph.R];
                graph.w = randi(5,graph.nL+graph.nR,1);
                %graph.w = [5*ones(graph.nL,1);ones(graph.nR,1)];
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                graph.E = [];
                for i=1:graph.nL
                    for j=1:graph.nR
                        if rand > 0.5
                            if abs(i-j) < randi(5) & (rand > P)
                                graph.E = [graph.E; i graph.nL+j];
                            end
                        else
                            if rand > P
                                graph.E = [graph.E; i graph.nL+j];
                            end
                        end
                    end
                end
                
                if isempty(graph.E)
                    graph.E = [1 graph.nL+1];
                end
                
                graph.nMC=grMinVerCover(graph.E,graph.w);
                h = grPlot([graph.V graph.w],graph.E,'g','%d','',1,graph.nMC);
                drawnow
                
                pause(1)
                close(h)
                
                x = sum(graph.w(1:graph.nL)) - sum(graph.w(graph.nMC));
                X = [X;x];
%                 hist(X); drawnow
                
                
                
                
                %grPlot(graph.V,graph.E,'g','%d','')
                %                 t = sprintf('Num. Edges: %d          M. Vertex Cover: %d',...
                %                     length(graph.E),length(graph.nMC));
                %                 title(t)
                %                 drawnow
                
%                 
%                 dis_L = graph.nL - length(unique(graph.E(:,1)));
%                 dis_R = graph.nR - length(unique(graph.E(:,2)));
%                 
                %x = -length(graph.nMC) + min(graph.nL-dis_L , graph.nR-dis_R);
%                 
%                 X_rel = [X_rel ; x/min(graph.nL-dis_L , graph.nR-dis_R)*100];
%                 
%                 
%                 hist(X);
%                 
%                 drawnow
                
                G = [G; graph];
                
                if x > x_max
                    graph_max = graph;
                    E_max = graph.E;
                    x_max = x
                    I_max = I;
                    J_max = J;
                    P_max = P;
                end
            end
        end
    end
end