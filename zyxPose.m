function poseMatrix = zyxPose(x,y,z,a,b,g)
    % Return Pose Matrix given x,y,z and Euler angles alpha (a), beta (b), gamma (g) of
    % x,y,z respectively
    % a,b,g in degrees
    a = deg2rad(a);
    b = deg2rad(b);
    g = deg2rad(g);
    ix = cos(g)*cos(b);
    iy = sin(g)*cos(b);
    iz = -sin(b);
    jx = cos(g)*sin(b)*sin(a)-sin(g)*cos(a);
    jy = sin(g)*sin(b)*sin(a)+cos(g)*cos(a);
    jz = cos(b)*sin(a);
    kx = cos(g)*sin(b)*cos(a)+sin(g)*sin(a);
    ky = sin(g)*sin(b)*cos(a)-cos(g)*sin(a);
    kz = cos(b)*cos(a);
    poseMatrix = [
        [ix, jx, kx, x],
        [iy, jy, ky, y],
        [iz, jz, kz, z],
        [0, 0, 0, 1],
    ];
end

