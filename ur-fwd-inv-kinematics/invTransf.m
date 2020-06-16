% Return inverse of transformation M
% M must be 4x4 orthogonal transformation
function M_inv = invTransf(M)
    R = M(1:3, 1:3);
    RT = transpose(R); % inverse = transpose, transpose has less calculations
    p = M(1:3, 4);
    M_inv = sym(eye(4));
    M_inv(1:3, 1:3) = RT;
    M_inv(1:3, 4) = -RT * p;
end

