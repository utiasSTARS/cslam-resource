function [x,y,t,w] = gen_random_trajectory(X0, x_lim, y_lim, v, w_max, yaw_max, N)
%gen_random_trajectory Generate a random trajectory
% X0 - x0, y0, t0, w0
% Use "hidden" w state (heading of motion, uncoupled from yaw of camera) to
% generate "realistic" (Dubbins-like) trajectories

x = zeros(N,1);
x(1) = X0(1);
y = zeros(N,1);
y(1) = X0(2);
t = zeros(N,1);
t(1) = X0(3);
w = zeros(N,1);
w(1) = X0(4);

x_mid = (x_lim(2) - x_lim(1))/2;
y_mid = (y_lim(2) - y_lim(1))/2;

for i=1:N-1
    
    dx = v*cos(w(i));
    dy = v*sin(w(i));
    
    while x(i) + dx < x_lim(1) || x(i) + dx > x_lim(2) || ...
            y(i) + dy < y_lim(1) || y(i) + dy > y_lim(2)
    
        % Set the heading towards the center
        w(i) = atan2(y_mid - (y(i)+dy), x_mid - (x(i)+dx));        
        dx = v*cos(w(i));
        dy = v*sin(w(i));
    end
    
    t(i+1) = t(i) - yaw_max + 2*rand()*yaw_max;
    w(i+1) = w(i) - w_max + 2*rand()*w_max;
    x(i+1) = x(i) + dx;
    y(i+1) = y(i) + dy;
    
end

end

