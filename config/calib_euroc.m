


T_C0toI = [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975;...
            0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768;...
            -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949;...
            0.0, 0.0, 0.0, 1.0];

T_C1toI = [0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556;...
            0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024;...
            -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038;...
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



