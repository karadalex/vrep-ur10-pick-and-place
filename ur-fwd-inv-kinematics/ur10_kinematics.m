%% Forward Kinematics
d = [0.1273, 0, 0, 0.163941, 0.1157, 0.0922];
L = [0, -0.612, -0.5723, 0, 0, 0];
a = [pi/2, 0, 0, pi/2, -pi/2, 0];

[M, M_joints] = fwdKinSym(L, d, a);


%% Inverse Kinematics
M_U_0 = pose(0,0,0,0,0,0);
M_U_TCP = pose(0.2,0.1,0.5,pi,0,pi/2);
M_6_TCP = pose(0,0,0,0,0,0);
M_0_6 = inv(M_U_0) * M_U_TCP * inv(M_6_TCP);
px = M_0_6(1,4); py = M_0_6(2,4); pz = M_0_6(3,4);
ix = M_0_6(1,1); iy = M_0_6(2,1);
jx = M_0_6(1,2); jy = M_0_6(2,2);

th = ones(6,8);

% theta 1
p_0_5 = M_0_6 * [0;0;-d(6);1];
th1_a = atan2(p_0_5(2), p_0_5(1));
r = sqrt(p_0_5(1)^2 + p_0_5(2)^2);
th1_b = acos(d(4)/r);
th(1,1:4) = th1_a + th1_b + pi/2;
th(1,5:8) = th1_a - th1_b + pi/2;


% theta 5
for i=1:4:8
    th1 = th(1,i);
    M_0_1 = eval(subs(M_joints(:,:,1), sym('th1'), th1));
    M_1_0 = eval(invTransf(M_0_1));
    M_1_6 = M_1_0 * M_0_6;
    
    ac5 = acos((M_1_6(3,4) - d(4)) / d(6));
    th(5,i:i+1) = real(ac5);
    th(5,i+2:i+3) = real(-ac5);
end

% theta 6
for i=1:2:8
    th1 = th(1,i);
    th5 = th(5,i);
    
    M_0_1 = eval(subs(M_joints(:,:,1), sym('th1'), th1));
    M_1_0 = eval(invTransf(M_0_1));
    M_1_6 = M_1_0 * M_0_6;
    M_6_1 = eval(invTransf(M_1_6));
    
    th(6, i:i+1) = atan2(-M_6_1(2,3)/sin(th5), M_6_1(1,3)/sin(th5));
end

% theta 3
for i=1:2:8
    th1 = th(1,i);
    th5 = th(5,i);
    th6 = th(6,i);
    
    M_0_1 = eval(subs(M_joints(:,:,1), sym('th1'), th1));
    M_4_5 = eval(subs(M_joints(:,:,5), sym('th5'), th5));
    M_5_6 = eval(subs(M_joints(:,:,6), sym('th6'), th6));
    
    M_1_0 = eval(invTransf(M_0_1));
    M_1_4 = M_1_0 * M_0_6 * inv(M_4_5 * M_5_6);
    p_1_3 = M_1_4 * [0;-d(4);0;1];
    p_1_3_norm = sqrt(p_1_3(1)^2 + p_1_3(2)^2 + p_1_3(3)^2);
    c3 = (p_1_3_norm^2 - L(2)^2 - L(3)^2) / (2*L(2)*L(3));
    s3 = sqrt(1-c3^2);
    
    th(3, i) = atan2(s3, c3);
    th(3, i+1) = atan2(-s3, c3);
end

% theta 2, theta 4
for i=1:8
    th1 = th(1,i);
    th5 = th(5,i);
    th6 = th(6,i);
    th3 = th(3,i);
    
    M_0_1 = eval(subs(M_joints(:,:,1), sym('th1'), th1));
    M_4_5 = eval(subs(M_joints(:,:,5), sym('th5'), th5));
    M_5_6 = eval(subs(M_joints(:,:,6), sym('th6'), th6));
    
    M_1_0 = eval(invTransf(M_0_1));
    M_1_4 = M_1_0 * M_0_6 * inv(M_4_5 * M_5_6);
    p_1_3 = M_1_4 * [0;-d(4);0;1];
    p_1_3_norm = sqrt(p_1_3(1)^2 + p_1_3(2)^2 + p_1_3(3)^2);
    
    % theta 2
    th(2, i) = -atan2(p_1_3(2), -p_1_3(1)) + asin(L(3)*sin(th3) / p_1_3_norm);
    
    % theta 4
    M_1_2 = eval(subs(M_joints(:,:,2), sym('th2'), th(2, i)));
    M_2_3 = eval(subs(M_joints(:,:,3), sym('th3'), th3));
    M_3_4 = invTransf(M_2_3) * invTransf(M_1_2) * M_1_4;
    th(4, i) = atan2(M_3_4(2,1), M_3_4(1,1));
end


%% Solutions validation
for i=1:8
    solution = th(:,i)';
    Mv = eval(subs(M, sym('th', [1, 6]), solution));
    Mv_U_TCP = M_U_0 * Mv * M_6_TCP;
    %disp(Mv_U_TCP - M_0_6);
end