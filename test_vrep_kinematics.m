%% Start connection to V-REP
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!
disp('Program started');
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

%% Test forward and inverse kinematics wrt to test reference frame
if (clientID>-1)
    disp('Connected to remote API server');

    % Command UR10 robot
    home = zeros(1,6);
    home(2) = -pi/2;
    home(4) = -pi/2;
    
    moveRobot(clientID, vrep, home);
    pause(5);
    
    angles = [pi, 0, -pi/2, pi/2, pi/2, 0];
    moveRobot(clientID, vrep, angles);
    pause(5);
    
    [M_0_6, M_joints_fwd] = fwdKinNum(L, d, a, angles);
    M_U_0 = pose(0,0,0,pi,0,0) * pose(0.2664, 0.2750, 0.819, -pi/2, 0, 0);
    M_6_TCP = pose(0, 0, 0.1606, 0, 0, 0);
    M_U_TCP = eval(M_U_0 * M_0_6 * M_6_TCP);
    disp(M_U_TCP);
    
    moveRobot(clientID, vrep, home);
    pause(5);
    
    % test again with invKin calculated angles
    M_0_6 = inv(M_U_0) * M_U_TCP * inv(M_6_TCP);
    angles = invKin(M_0_6, M_joints, L, d, a);
    moveRobot(clientID, vrep, angles(:,8));
    pause(5);
    
    moveRobot(clientID, vrep, home);
    pause(5);

else
    disp('Failed connecting to remote API server');
end

%% Close the connection to V-REP:  
if (clientID>-1)
    vrep.simxFinish(clientID);
end
vrep.delete(); % call the destructor!

disp('Program ended');