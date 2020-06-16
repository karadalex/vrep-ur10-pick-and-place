function Jv = geometricJacobian(M_total, M_joints, dof)
    p_end = M_total(1:3, 4);
    Jv = sym(zeros(6, dof));
    M_0_i = sym(eye(4));
    for i = 1:1:dof
        M_0_i = M_0_i * M_joints(:,:,i);
        k_0_i = M_0_i(1:3, 3);
        p_0_i = M_0_i(1:3, 4);
        Jv(1:3, i) = simplify(cross(k_0_i, p_end-p_0_i));
        Jv(4:6, i) = k_0_i;
    end
end

