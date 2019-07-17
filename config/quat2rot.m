function [A]=quat2rot(q)

q = q/norm(q);
A = eye(3) - 2 * q(4) * skewsymm(q(1:3)) + 2 * skewsymm(q(1:3))^2; % Identical to MARS report
% A = eye(3)*(2*q(4)^2 - 1) - 2*q(4)*skewsymm(q(1:3)) + 2*q(1:3)*q(1:3)'; % from MARS report


function C = skewsymm(X1)

% generates skew symmetric matrix

C = [0      , -X1(3) ,  X1(2)
    X1(3) , 0      , -X1(1)
    -X1(2) , X1(1)  ,     0];


