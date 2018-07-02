% Given a angle, first normalize it to [-pi, pi)
% then evaluate the derivative of the rotation matrix

function dR = angle_to_SO2_derivative(theta)
    theta = wrapToPi(theta);
    dR = zeros(2);
    dR(1,1) = -sin(theta);
    dR(1,2) = -cos(theta);
    dR(2,1) = cos(theta);
    dR(2,2) = -sin(theta);
end