function nMC=grMinVerCoverGurobi(E,d,gurobi_flag)
% Function nMC=grMinVerCover(E,d) solve the minimal vertex cover problem
% for bipartite graphs.
% Input parameters: 
%   E(m,2) - the edges of graph;
%     1st and 2nd elements of each row is numbers of vertexes;
%     m - number of edges.
%   d(n) (optional) - the weights of vertexes,
%     n - number of vertexes.
%     If we have only 1st parameter E, then all d=1.
% Output parameter:
%   nMC - the list of the numbers of vertexes included 
%     in the minimal (weighted) vertex cover.
% Required the Optimization Toolbox v.3.0.1 or over.
% Modified (by Matthew Giamou) version of code by:
% Author: Sergii Iglin
% e-mail: siglin@yandex.ru
% personal page: http://iglin.exponenta.ru

% ============= Input data validation ==================
if nargin<1
  error('There are no input data!')
end
[m,n,E] = grValidation(E); % E data validation
if nargin<2 % we may only 1st parameter
  d=ones(n,1); % all weights =1
else
  d=d(:); % reshape to vector-column
  if length(d)<n % the poor length
    error('The length of the vector d is poor!')
  else
    n=length(d); % Number of Vertices
  end
end

% ============= Parameters of integer LP problem ==========
A=sparse(n,m); % for incidence matrix
A(E(:,1:2)+repmat(([1:m]'-1)*n,1,2))=1; % we fill the incidence matrix
%options=optimset('bintprog'); % the default options
%options.Display='off'; % we change the output

if gurobi_flag
    xmin=glinprog(d,-A',-ones(m,1),[],[],zeros(n,1),ones(n,1));
else
    xmin=linprog(d,-A',-ones(m,1),[],[],sparse(n,1),[]);
end
nMC=find(xmin); % the answer - numbers of vertexes
return