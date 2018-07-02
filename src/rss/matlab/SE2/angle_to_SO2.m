% Given a angle, first normalize it to [-pi, pi)
% then convert it to rotation matrix

function R = angle_to_SO2(theta)
    theta = wrapToPi(theta);
    R = zeros(2);
    R(1,1) = cos(theta);
    R(1,2) = -sin(theta);
    R(2,1) = sin(theta);
    R(2,2) = cos(theta);
end