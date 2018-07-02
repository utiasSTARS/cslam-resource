function A = sensor_overlap(p1, r1, t1, p2, r2, t2, display_result)
%sensor_overlap Compute overlapping area of circular sector sensor footprints
%   p1 and p2 are [x,y,theta] of robot pose 
%   r and t are range and fov of sensor

if nargin < 7
    display_result = 0;
end
% Construct each approximating polygon
P1 = circular_sector(p1, r1, t1);
P2 = circular_sector(p2, r2, t2);
clear S;
S(1).P(1) = P1;
S(2).P(1) = P2;
accuracy = 1e-6;
G = Polygons_intersection(S, display_result, accuracy);
A = G(1).area;

end

function P = circular_sector(p, r, t, N)

if nargin < 4
    N = ceil(t/0.03);
end

P.x =[p(1) p(1)+r*cos(p(3)+linspace(-t/2, t/2, N)) p(1)];
P.y =[p(2) p(2)+r*sin(p(3)+linspace(-t/2, t/2, N)) p(2)];
P.hole = 0;

end