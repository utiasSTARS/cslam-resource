%% Generate an example graph from a simulation with simple random kinematics 
clear;

%% Params
% Traj params
X0 = [0, 0, pi/4, pi/4];
x_lim = [0, 20];
y_lim = [0, 20];
v = 1.0;
w_max = 0.2*pi/2; 
yaw_max = 0.2*pi/2; 
N = 100;

% Sensor params
r = 1.0;
fov = pi/2;
fov_area = 0.5*fov*r^2;
min_overlap = 0.5*fov_area;

%% Generate and plot two random trajectories
[x1,y1,t1,w1] = gen_random_trajectory(X0, x_lim, y_lim, v, w_max, yaw_max, N);
[x2,y2,t2,w2] = gen_random_trajectory(X0, x_lim, y_lim, v, w_max, yaw_max, N);

figure;
hold on;
plot(x1, y1, 'b-o');
plot(x2, y2, 'r--o');
xlabel('x'); ylabel('y');
title('Random Trajectories');
legend('Agent 1', 'Agent 2');
hold off;

%% Compute the bipartite graph using overlapping FOVs
graph.E = []; % Store edges
for idx=1:size(x1)
    p1 = [x1(idx) y1(idx) t1(idx)];
    for jdx=1:size(x2)
        if sqrt( (x1(idx)-x2(jdx))^2 + (y1(idx) - y2(jdx))^2 ) < r
            p2 = [x2(jdx) y2(jdx) t2(jdx)];
            A = sensor_overlap(p1, r, fov, p2, r, fov);
            if A > min_overlap
                graph.E = [graph.E; idx jdx+N];
            end
        end
    end 
end

%% Solve ODEP
% Uniform measuerment sizes
W = ones(2*N,1);
gurobi_flag = true;
[graph.nMC, cover] = solve_odep(graph.E, W, gurobi_flag);
% Perturb y2 slightly to avoid an error at the origin (can't have identical
% vertices)
grPlot([[x1; x2] [y1; y2+0.01]], graph.E, 'g', '%d', '', 1, cover);
drawnow

%% Improvement from solving OSEP
n_naive = min(size(unique(graph.E(:,1)), 1), size(unique(graph.E(:,2)), 1));
fprintf('N = %d', N);
fprintf('N scans naive: %d \nN scans MinVerCover: %d', n_naive, size(cover, 1));
