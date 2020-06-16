function [M_total, M_joints] = fwdKinSym(L, d, a)
    % Create/convert to symbolic variables
    L = sym(L);
    d = sym(d);
    a = sym(a);
    theta = sym('th', [1, 6]);
    
    % Initialize transformation matrices
    M_joints = sym('M', [4 4 6]);
    M_total = eye(4);
    M_total = sym(M_total);
    
    for i = 1:1:6
      TL = sym(eye(4));
      TL(1,4) = L(i);
      Td = sym(eye(4));
      Td(3,4) = d(i);
      Rth = [
        [cos(theta(i)), -sin(theta(i)), 0, 0],
        [sin(theta(i)), cos(theta(i)), 0, 0],
        [0,             0,             1, 0],
        [0,             0,             0, 1],
      ];
      Ra = [
        [1, 0,         0,          0],
        [0, cos(a(i)), -sin(a(i)), 0],
        [0, sin(a(i)), cos(a(i)),  0],
        [0, 0,         0,          1],
      ];
      M_joints(:,:,i) = Td * Rth * TL * Ra;
      disp(M_joints(:,:,i))
      M_total = simplify(M_total) * M_joints(:,:,i);
    end
    M_total = simplify(M_total);
end