%% Load target positions, orientations and paths
referenceFrames
addpath('./ur-fwd-inv-kinematics');

path = zeros(9,6);

home = zeros(1,6);
home(2) = -pi/2;
home(4) = -pi/2;

path(1,:) = home;
path(10,:) = home;
    
% solve IK for start and target positions of bottles
k = 2;
for i=1:4
    th = invKin(M_0_6_bis(:,:,i), M_joints, L, d, a);
    path(k, :) = th(:,1);
    k = k + 1;
    th = invKin(M_0_6_bit(:,:,i), M_joints, L, d, a);
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
    for i = 1:10
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