%% Load target positions, orientations and paths
referenceFrames
addpath('./ur-fwd-inv-kinematics');

path = zeros(26,6);

home = zeros(1,6);
home(2) = -pi/2;
home(4) = -pi/2;

path(1,:) = home;
path(26,:) = home;
    
% solve IK for start and target positions of bottles
k = 2;
for i=1:4
    % PICK
    % Above starting point
    th = invKin(M_0_6_bis(:,:,i)*pose(0,0,-0.2,0,0,0), M_joints, L, d, a);
    path(k, :) = th(:,1);
    k = k + 1;
    
    % At starting point
    th = invKin(M_0_6_bis(:,:,i), M_joints, L, d, a);
    path(k, :) = th(:,1);
    k = k + 1;
    
    % Above starting point (after having picked the bottle)
    th = invKin(M_0_6_bis(:,:,i)*pose(0,0,-0.4,0,0,0), M_joints, L, d, a);
    path(k, :) = th(:,1);
    k = k + 1;
    
    % PLACE
    % Approaching target point
    th = invKin(M_0_6_bit(:,:,i)*pose(0,0,-0.5,0,0,0), M_joints, L, d, a);
    path(k, :) = th(:,1);
    k = k + 1;
    
    % At target point
    th = invKin(M_0_6_bit(:,:,i), M_joints, L, d, a);
    path(k, :) = th(:,1);
    k = k + 1;
    
    % Leaving target point (after having placed the bottle)
    th = invKin(M_0_6_bit(:,:,i)*pose(0,0,-0.5,0,0,0), M_joints, L, d, a);
    path(k, :) = th(:,1);
    k = k + 1;
end

%% Start connection to V-REP
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!
disp('Program started');
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);


%% Send commands to V-REP remoteApiServer
if (clientID>-1)
    disp('Connected to remote API server');

    % Command UR10 robot    
    for i = 1:26
        moveRobot(clientID, vrep, path(i,:));
        pause(5);
    end

else
    disp('Failed connecting to remote API server');
end


%% Close the connection to V-REP:  
if (clientID>-1)
    vrep.simxFinish(clientID);
end
vrep.delete(); % call the destructor!

disp('Program ended');