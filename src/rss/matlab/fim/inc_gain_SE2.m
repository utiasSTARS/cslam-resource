function gain = inc_gain_SE2(MSP, R, S, new_edges, new_edges_prob, fix_vertex)
assert(size(new_edges,1) == size(new_edges_prob, 1));

if size(new_edges,1) == 0
   gain = 0;
   return;
end

information = eye(3);

% form the 3(n-1) by 3m matrix A
% here n is the number of poses, m is the number of new edges (set_size)
% n-1 because anchoring one vertex

% A = J^T I^(1/2), where J is the 3m by 3(n-1) stacked jacobian, I is the 3m
% by 3m block diagnoal information matrix
I = [];
J = [];

for i = 1:size(new_edges, 1)
    p = new_edges_prob(i);
    % compute Jacobian corresponding to edge i
    idi = MSP.vid_to_id(new_edges(i,1)); % convert from g2o index to matlab index
    idj = MSP.vid_to_id(new_edges(i,2));
    xi = [MSP.id_to_x(idi); MSP.id_to_y(idi); MSP.id_to_theta(idi)];
    xj = [MSP.id_to_x(idj); MSP.id_to_y(idj); MSP.id_to_theta(idj)];
    zij = [new_edges(i, 3); new_edges(i, 4); new_edges(i, 5)]; % observed translation and rotation

    Jij = jacobian_SE2(MSP, idi, idj, xi, xj, zij, fix_vertex);
    J = [J; Jij];
    I = blkdiag(I, p*information);   
end

I_half = sqrt(I); % assume diagonal I
A = J' * I_half;

% test
% assert(size(J,1) == 3*size(new_edges,1));
% assert(size(J,2) == 3*(MSP.N-1));
% assert(size(A,1) == 3*(MSP.N-1));
% assert(size(A,2) == 3*size(new_edges,1));
% assert(size(I,1) == 3*size(new_edges,1));

gain = logdet_inc(R, S, A, true);

end