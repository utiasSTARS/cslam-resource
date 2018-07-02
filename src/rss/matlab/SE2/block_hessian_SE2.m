% Compute the hessian blocks H_ii, H_ij, and Hji corresponding to an edge
% at the given estimate
% Use notation as in: 
% http://domino.informatik.uni-freiburg.de/teaching/ws11/robotics2/pdfs/ls-slam-tutorial.pdf
% See especially Algorithm 2 and Appendix

function [Hii, Hij, Hjj] = block_hessian_SE2(idi, idj, xi, xj, zij, information)
% lots of sanity check
assert(idi ~= idj);
assert(size(xi,1) == size(xj,1));
assert(size(xi,1) == size(zij,1));
assert(size(xi,1) == size(information,1));

dimension = size(xi,1); 
assert(dimension == 3); % Only work with SE2!


% compute jacobian Aij and Bij
[Aij, Bij] = block_jacobian_SE2(xi, xj, zij);

Hii = Aij' * information * Aij;
Hjj = Bij' * information * Bij;
Hij = Aij' * information * Bij;


end