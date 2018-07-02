% validate that a given solution is valid
% i.e. 
% all selected vertices are from MSP.Vlist
% each selected candidate is connected to at least one selected vertex
function validate_solution(MSP, g)

for i = 1:size(g.VC)
   v = g.VC(i);
   assert(ismember(v, MSP.Vlist), 'Selected vertex not in Vlist!');
end

for i = 1:size(g.edges, 1)
   edge = g.edges(i,:);
   u = g.edges(i,1);
   v = g.edges(i,2);
   
   if ~ismember(edge, MSP.g_init.edges, 'rows')
       assert(ismember(u, g.VC) || ismember(v, g.VC), 'Selected edge not covered by vertex!')
   end
   
end




end
