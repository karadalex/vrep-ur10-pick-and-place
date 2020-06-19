function res = moveRobot(clientID, vrep, position)
    [res, retInts, retFloats, retStrings, retBuffer]=vrep.simxCallScriptFunction(clientID,'UR10',vrep.sim_scripttype_childscript,'commandUR10',[],position,[],[],vrep.simx_opmode_blocking);
    if (res==vrep.simx_return_ok)
        fprintf('Code execution returned: %s\n',retStrings);
    else
        fprintf('Remote function call failed\n');
    end
end

