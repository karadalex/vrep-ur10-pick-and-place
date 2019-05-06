% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!

%% Start connection to V-REP
disp('Program started');
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);


%% Load target positions, orientations and paths
referenceFrames


%% Send commands to V-REP remoteApiServer
if (clientID>-1)
    disp('Connected to remote API server');

    % Command UR10 robot
    targetPosition = [90*pi/180, 90*pi/180, -90*pi/180, 90*pi/180, 90*pi/180, 90*pi/180];
    [res, retInts, retFloats, retStrings, retBuffer]=vrep.simxCallScriptFunction(clientID,'UR10',vrep.sim_scripttype_childscript,'commandUR10',[],targetPosition,[],[],vrep.simx_opmode_blocking);
    if (res==vrep.simx_return_ok)
        fprintf('Code execution returned: %s\n',retStrings);
    else
        fprintf('Remote function call failed\n');
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