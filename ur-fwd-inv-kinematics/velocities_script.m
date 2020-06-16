%% Forward Kinematics section
% execute forward kinematics script
forward_script

%% Calculate Jacobian for 6 dof
% Available in workspace: a, d, L, M, M_joints, orient, pos, test_angles
Jv = geometricJacobian(M, M_joints, 6);
disp(Jv);