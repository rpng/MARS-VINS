


imu_q_cam = [0,         0,   -0.7071,    0.7071];
imu_q_cam_right = [0.013617636555835  -0.013384815447097  -0.712501629148587   0.701396931238432];


R_C0toC1 = quat2rot(imu_q_cam)'*quat2rot(imu_q_cam_right);
q_C0toC1 = rot2quat(R_C0toC1);

fprintf('cam_l_q_cam_r = \n')
fprintf('%.8f  %.8f  %.8f  %.8f\n',q_C0toC1(1,1),q_C0toC1(2,1),q_C0toC1(3,1),q_C0toC1(4,1))



imu_p_cam = [.00157, -.13336, .0235]';
imu_p_cam_right = [0.009804716553368   0.134603406534874  -0.002273390520198]';


R_C0toI = quat2rot(imu_q_cam);
p_C1inC0 = R_C0toI'*(imu_p_cam_right-imu_p_cam);
p_C0inC1 = R_C1toI'*(imu_p_cam-imu_p_cam_right);


fprintf('cam_l_p_cam_r = \n')
fprintf('%.8f  %.8f  %.8f\n',p_C1inC0(1,1),p_C1inC0(2,1),p_C1inC0(3,1))
