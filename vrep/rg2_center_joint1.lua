function sysCall_init()
    openCloseMotor=sim.getObjectHandle('RG2_openCloseJoint')
    PID_P=0.1
end

function sysCall_jointCallback(inData)
    local errorValue=(-sim.getJointPosition(openCloseMotor)/2)-inData.currentPos
    local ctrl=errorValue*PID_P
    
    local maxVelocity=ctrl/inData.dynStepSize
    if (maxVelocity>inData.velUpperLimit) then
        maxVelocity=inData.velUpperLimit
    end
    if (maxVelocity<-inData.velUpperLimit) then
        maxVelocity=-inData.velUpperLimit
    end
    local forceOrTorqueToApply=inData.maxForce

    local outData={}
    outData.velocity=maxVelocity
    outData.force=forceOrTorqueToApply
    return outData
end