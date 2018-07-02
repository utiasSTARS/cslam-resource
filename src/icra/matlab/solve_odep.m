function [n_mvc_fast, cover] = solve_odep(E, W, gurobi_flag)
%solve_odep Return a list containing a communication cost for each 
% component of (E,W)

if nargin < 3
    gurobi_flag = 0;
end

connected_v = unique(E);
E_re_indexed = zeros(size(E));
W_re_indexed = zeros(size(connected_v));
for idx = 1:length(connected_v)
    W_re_indexed(idx) = W(connected_v(idx));
end
for idx = 1:size(E, 1)
    E_re_indexed(idx,1) = find(connected_v == E(idx,1));
    E_re_indexed(idx,2) = find(connected_v == E(idx,2));
end
comp_fast = grComp(E_re_indexed);
unique_comp_fast = unique(comp_fast);
n_mvc_fast = zeros(1, length(unique_comp_fast));
cover = [];
for idx = 1:length(unique_comp_fast)
    comp_i = unique_comp_fast(idx);
    index_idx = find(comp_fast == comp_i);
    E_i = E_re_indexed(and(ismember(E_re_indexed(:,1), index_idx), ismember(E_re_indexed(:,2),index_idx)), :);
    cover_idx = grMinVerCoverGurobi(E_i, W_re_indexed, gurobi_flag);
    cover = [cover; cover_idx];
    n_mvc_fast(idx) = sum(W_re_indexed(cover_idx));
end

end

