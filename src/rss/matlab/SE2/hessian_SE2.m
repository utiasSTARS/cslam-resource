% Compute the hessian matrix H
% This function takes in a MSP describing the problem, an edge set
% and a anchor vertex
function hessian = hessian_SE2(MSP, edges, edges_prob, fix_vertex)
assert(size(edges,1) == size(edges_prob, 1));
dimension = 3;
information = eye(dimension); % hardcoded information matrix for now
% assert(size(edges, 1) >= MSP.N-1)
hessian = sparse(dimension * MSP.N, dimension * MSP.N);

for i =1:size(edges, 1)
   p = edges_prob(i);
   idi = MSP.vid_to_id(edges(i,1)); % convert from g2o index to matlab index
   idj = MSP.vid_to_id(edges(i,2));
   xi = [MSP.id_to_x(idi); MSP.id_to_y(idi); MSP.id_to_theta(idi)];
   xj = [MSP.id_to_x(idj); MSP.id_to_y(idj); MSP.id_to_theta(idj)];
   zij = [edges(i, 3); edges(i, 4); edges(i, 5)]; % observed translation and rotation from i to j
   
   [Hii, Hij, Hjj] = block_hessian_SE2(idi, idj, xi, xj, zij, p * information);
   
   i_start = (idi-1) * dimension + 1;
   i_end = idi * dimension;
   j_start = (idj-1) * dimension + 1;
   j_end = idj * dimension;
   
   % diagonal blocks
   hessian(i_start:i_end, i_start:i_end) = hessian(i_start:i_end, i_start:i_end) + Hii;
   hessian(j_start:j_end, j_start:j_end) = hessian(j_start:j_end, j_start:j_end) + Hjj;
   % non-diagonal blocks
   hessian(i_start:i_end, j_start:j_end) = hessian(i_start:i_end, j_start:j_end) + Hij;
   hessian(j_start:j_end, i_start:i_end) = hessian(j_start:j_end, i_start:i_end) + Hij';
end

% remove the block-row and block-column corresponding to the fixed vertex
fix_start = (fix_vertex-1) * dimension + 1;
fix_end = fix_vertex * dimension;

range = [1:fix_start-1 fix_end+1:dimension*MSP.N];
hessian = hessian(range, range);

end