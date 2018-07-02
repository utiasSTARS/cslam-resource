% Compute the jacobian blocks Aij and Bij corresponding to an edge
% at the given estimate
% Use notation as in: 
% http://domino.informatik.uni-freiburg.de/teaching/ws11/robotics2/pdfs/ls-slam-tutorial.pdf
% See especially Appendix

function [Aij, Bij] = block_jacobian_SE2(xi, xj, zij)
dimension = size(xi,1);
assert(dimension == 3);

Aij = zeros(dimension);
Bij = zeros(dimension);

% translational part
ti = xi(1:2);
tj = xj(1:2);
tij = zij(1:2);

% convert angle to rotation matrix
Ri = angle_to_SO2(xi(3));
Rj = angle_to_SO2(xj(3));
Rij = angle_to_SO2(zij(3));
dRi = angle_to_SO2_derivative(xi(3));

Aij = [-Rij'*Ri' Rij'*dRi'*(tj-ti);
      zeros(1,2) -1];
Bij = [Rij'*Ri' zeros(2,1);
      zeros(1,2) 1];


end