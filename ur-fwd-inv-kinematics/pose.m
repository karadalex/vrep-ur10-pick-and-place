function M = pose(x,y,z,roll,pitch,yaw)
    Rot_r = [cos(roll), -sin(roll), 0;
             sin(roll), cos(roll), 0;
             0,        0,    1];
    Rot_p = [cos(pitch), 0, sin(pitch);
             0,      1,    0;
             -sin(pitch),0, cos(pitch)];
    Rot_y = [1,    0,     0;
             0, cos(yaw), -sin(yaw);
             0, sin(yaw), cos(yaw)];
    Rot_rpy = Rot_r * Rot_p * Rot_y;
    M(1:3,1:3) = Rot_rpy;
    M(1:4,4) = [x; y; z; 1];
end

