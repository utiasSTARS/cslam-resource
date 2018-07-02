% compute the 3 by 3N full Jacobian matrix corresponding to measurement zij
% where N is the total number of vertices/poses.
function J = jacobian_SE2(MSP, idi, idj, xi, xj, zij, fix_vertex)
dimension = 3;
assert(size(xi,1) == dimension);
J = zeros(dimension, dimension * MSP.N);

i_start = (idi-1) * dimension + 1;
i_end = idi * dimension;
j_start = (idj-1) * dimension + 1;
j_end = idj * dimension;

[Aij, Bij] = block_jacobian_SE2(xi, xj, zij);

J(:,i_start:i_end) = Aij;
J(:,j_start:j_end) = Bij;

% remove block corresponding to fix vertex
fix_start = (fix_vertex-1) * dimension + 1;
fix_end = fix_vertex * dimension;

range = [1:fix_start-1 fix_end+1:dimension*MSP.N];
J = J(:, range);

end