%% Define Reference frames
addpath('./ur-fwd-inv-kinematics');
d = [0.1273, 0, 0, 0.163941, 0.1157, 0.0922];
L = [0, -0.612, -0.5723, 0, 0, 0];
a = [pi/2, 0, 0, pi/2, -pi/2, 0];
[M_0_6, M_joints] = fwdKinSym(L, d, a);
M_6_TCP = pose(0, 0, 0.1606, 0, 0, 0);
M_TCP_6 = invTransf(M_6_TCP);

%M_U_0 = zyxPose(-0.2664, -0.2750, 0.8, 0, 0, -90);
M_U_0 = pose(0,0,0,pi,0,0) * pose(0.2664, 0.2750, 0.819, -pi/2, 0, 0);
%M_U_0 = zyxPose(0, 0, 0, 0, 0, -180);
M_0_U = invTransf(M_U_0);

% bottle initial poses
M_U_bis(:,:,1) = pose(0.7080, -0.1830, 0.8780, 0, 0, -pi);
M_U_bis(:,:,2) = pose(0.7080, -0.2670, 0.8780, 0, 0, -pi);
M_U_bis(:,:,3) = pose(0.7920, -0.1830, 0.8780, 0, 0, -pi);
M_U_bis(:,:,4) = pose(0.7920, -0.2670, 0.8780, 0, 0, -pi);
% Convert poses w.r.t. robot base {0} and end-effector {6}
for i = 1:1:4
    M_0_6_bis(:,:,i) = double(M_0_U * M_U_bis(:,:,i) * M_TCP_6);
end
M_0_6_bis = double(M_0_6_bis);

% bottle target poses
M_U_bit(:,:,1) = pose(-0.9230, -0.05, 1.6920, 0, -pi/2, pi);
M_U_bit(:,:,2) = pose(-0.9230, -0.25, 1.6920, 0, -pi/2, pi);
M_U_bit(:,:,3) = pose(-0.9230, -0.45, 1.6920, 0, -pi/2, pi);
M_U_bit(:,:,4) = pose(-0.9230, -0.65, 1.6920, 0, -pi/2, pi);
% Convert poses w.r.t. robot base {0} and end-effector {6}
for i = 1:1:4
    M_0_6_bit(:,:,i) = M_0_U * M_U_bit(:,:,i) * M_TCP_6;
end
M_0_6_bit = double(M_0_6_bit);