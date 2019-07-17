


T_C0toI = [-0.99952504,  0.00750192, -0.02989013,  0.04557484;...
             0.02961534, -0.03439736, -0.99896935, -0.07116180;...
            -0.00852233, -0.99938008,  0.03415885, -0.04468125;...
            0.0, 0.0, 0.0, 1.0];

T_C1toI = [-0.99951105,  0.00810408, -0.03019914, -0.05545634;...
             0.03029912,  0.01251164, -0.99946257, -0.06925002;...
            -0.00772188, -0.99988889, -0.01275107, -0.04745286;...
            0.0, 0.0, 0.0, 1.0];



R_C0toI = T_C0toI(1:3,1:3);
R_C1toI = T_C1toI(1:3,1:3);

p_C0inI = T_C0toI(1:3,4);
p_C1inI = T_C1toI(1:3,4);

q_C0toI = rot2quat(R_C0toI);
q_C1toI = rot2quat(R_C1toI);


% calculate flip direction of ori and pos
q_ItoC0 = rot2quat(R_C0toI');
q_ItoC1 = rot2quat(R_C1toI');

p_IinC0 = -R_C0toI'*p_C0inI;
p_IinC1 = -R_C0toI'*p_C1inI;

% relative between cameras
q_C0toC1 = rot2quat(R_C1toI'*R_C0toI);
q_C1toC0 = rot2quat(R_C0toI'*R_C1toI);
p_C1inC0 = R_C0toI'*(p_C1inI-p_C0inI);
p_C0inC1 = R_C1toI'*(p_C0inI-p_C1inI);


fprintf('imu_q_cam = \n')
fprintf('%.8f, %.8f, %.8f, %.8f\n',q_C0toI(1,1),q_C0toI(2,1),q_C0toI(3,1),q_C0toI(4,1))
fprintf('imu_p_cam = \n')
fprintf('%.8f, %.8f, %.8f\n',p_C0inI(1,1),p_C0inI(2,1),p_C0inI(3,1))


fprintf('imu_q_cam_right = \n')
fprintf('%.8f, %.8f, %.8f, %.8f\n',q_C1toI(1,1),q_C1toI(2,1),q_C1toI(3,1),q_C1toI(4,1))
fprintf('imu_p_cam_right = \n')
fprintf('%.8f, %.8f, %.8f\n',p_C1inI(1,1),p_C1inI(2,1),p_C1inI(3,1))


fprintf('cam_l_q_cam_r = \n')
fprintf('%.8f, %.8f, %.8f, %.8f\n',q_C0toC1(1,1),q_C0toC1(2,1),q_C0toC1(3,1),q_C0toC1(4,1))
fprintf('cam_l_p_cam_r = \n')
fprintf('%.8f, %.8f, %.8f\n',p_C1inC0(1,1),p_C1inC0(2,1),p_C1inC0(3,1))



