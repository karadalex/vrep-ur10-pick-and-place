%% Define Reference frames
addpath('./fwd-inv-kinematics');
L = [0, 0.612, 0.572, 0, 0, 0];
d = [0.128, 0, 0, 0.164, 0.116, 0.092];
a = [0, -pi/2, 0, 0, pi/2, -pi/2];
[M_0_6, M_joints] = fwdKinSym(L, d, a);
M_6_TCP = zyxPose(0, 0, 0.1606, 0, 0, 0);

M_U_0 = zyxPose(-0.2664, -0.2750, 0.8, 0, 0, -90);

% bottle initial poses
M_U_bis(:,:,1) = zyxPose(0.7080, -0.1830, 0.8440, 0, 0, 0);
M_U_bis(:,:,2) = zyxPose(0.7080, -0.2670, 0.8440, 0, 0, 0);
M_U_bis(:,:,3) = zyxPose(0.7920, -0.1830, 0.8440, 0, 0, 0);
M_U_bis(:,:,4) = zyxPose(0.7920, -0.2670, 0.8440, 0, 0, 0);

% bottle target poses
M_U_bit(:,:,1) = zyxPose(-0.9230, -0.05, 1.6920, 0, 89.98, 0);
M_U_bit(:,:,2) = zyxPose(-0.9230, -0.25, 1.6920, 0, 89.98, 0);
M_U_bit(:,:,3) = zyxPose(-0.9230, -0.45, 1.6920, 0, 89.98, 0);
M_U_bit(:,:,4) = zyxPose(-0.9230, -0.65, 1.6920, 0, 89.98, 0);