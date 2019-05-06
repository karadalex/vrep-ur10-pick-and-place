%% Load target positions, orientations and paths
% !! TIME CONSUMING: 
% run once and save in workspace
referenceFrames
generalSolutions = invKinSym(M_joints);
% solve IK for start and target positions of bottles
startSol = zeros(64,6,4);
targetSol = zeros(64,6,4);
for i = 1:1:4
    startSol(:,:,i) = ikSolutionSet(M_0_6_bis(:,:,i), generalSolutions);
    targetSol(:,:,i) = ikSolutionSet(M_0_6_bit(:,:,i), generalSolutions);
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
    targetPosition = [90*pi/180, 90*pi/180, -90*pi/180, 90*pi/180, 90*pi/180, 90*pi/180];
    moveRobot(targetPosition)

else
    disp('Failed connecting to remote API server');
end


%% Close the connection to V-REP:  
if (clientID>-1)
    vrep.simxFinish(clientID);
end
vrep.delete(); % call the destructor!

disp('Program ended');