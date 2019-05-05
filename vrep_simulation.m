% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!

function vrep_simulation()
    disp('Program started');
    % vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

    if (clientID>-1)
        disp('Connected to remote API server');

        % Run default UR10 script
        targetPosition = [-90*pi/180, 90*pi/180, -90*pi/180, 90*pi/180, 90*pi/180, 90*pi/180];
        [res, retInts, retFloats, retStrings, retBuffer]=vrep.simxCallScriptFunction(clientID,'UR10',vrep.sim_scripttype_childscript,'commandUR10',[],targetPosition,[],[],vrep.simx_opmode_blocking);
        if (res==vrep.simx_return_ok)
            fprintf('Code execution returned: %s\n',retStrings);
        else
            fprintf('Remote function call failed\n');
        end


        % Now close the connection to V-REP:    
        vrep.simxFinish(clientID);
    else
        disp('Failed connecting to remote API server');
    end
    vrep.delete(); % call the destructor!
    
    disp('Program ended');
end