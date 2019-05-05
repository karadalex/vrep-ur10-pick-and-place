function sysCall_init() 
    model=sim.getObjectAssociatedWithScript(sim.handle_self)
    maxTraces=sim.getScriptSimulationParameter(sim.handle_self,'maxTraces')
    minPositionalVariation=sim.getScriptSimulationParameter(sim.handle_self,'minPositionalVariation')
    minAngularVariation=sim.getScriptSimulationParameter(sim.handle_self,'minAngularVariation')
    forceEveryXFrameRecording=sim.getScriptSimulationParameter(sim.handle_self,'forceEveryXFrameRecording')
    if (tonumber(maxTraces)==nil) then
        maxTraces=10
    end
    if (tonumber(minPositionalVariation)==nil) then
        minPositionalVariation=0.05
    end
    if (tonumber(minAngularVariation)==nil) then
        minAngularVariation=10*math.pi/180
    end
    if (tonumber(forceEveryXFrameRecording)==nil) then
        forceEveryXFrameRecording=1
    end
    maxTraces=math.floor(maxTraces)
    if (maxTraces<1) then maxTraces=1 end
    if (maxTraces>1000) then maxTraces=1000 end
    minPositionalVariation=math.abs(minPositionalVariation)
    minAngularVariation=math.abs(minAngularVariation)
    forceEveryXFrameRecording=math.floor(forceEveryXFrameRecording)
    if (forceEveryXFrameRecording<0) then forceEveryXFrameRecording=0 end
    
    originalShapeHandle=sim.getObjectParent(model)
    originalParent=originalShapeHandle
    sim.setObjectMatrix(model,sim.handle_parent,sim.buildIdentityMatrix())
    sim.setObjectParent(model,-1,true)
    copiedShapeHandles={}
    initialVisibilityLayers=sim.getInt32Parameter(sim.intparam_visible_layers)
    nextIndex=1
    frame=0
    if (originalShapeHandle~=-1) then
        if (sim.getObjectType(originalShapeHandle)==sim.object_shape_type) then
            vertices,indices=sim.getShapeMesh(originalShapeHandle)
            local m=sim.getObjectMatrix(originalShapeHandle,-1)
            for i=1,#vertices/3,1 do
                local v={vertices[3*(i-1)+1],vertices[3*(i-1)+2],vertices[3*(i-1)+3]}
                v=sim.multiplyVector(m,v)
                vertices[3*(i-1)+1]=v[1]
                vertices[3*(i-1)+2]=v[2]
                vertices[3*(i-1)+3]=v[3]                
            end
            local h=sim.createMeshShape(0,0,vertices,indices)
            local m2=sim.getObjectMatrix(h,-1)
            local mInv=simGetInvertedMatrix(m)
            local mTransf=sim.multiplyMatrices(mInv,m2)
            sim.setObjectParent(h,model,true)
            sim.setObjectInt32Parameter(h,sim.objintparam_visibility_layer,initialVisibilityLayers)
            local p=sim.getObjectProperty(h)
            p=sim.boolOr32(p,sim.objectproperty_selectable)-sim.objectproperty_selectable
            sim.setObjectProperty(h,p)
            sim.setObjectSpecialProperty(h,0)
            sim.setShapeColor(colorCorrectionFunction(h),'',0,{0.2,0.9,0})
            sim.setObjectInt32Parameter(h,sim.shapeintparam_wireframe,1)
            table.insert(copiedShapeHandles,h)
            if (maxTraces>1) then
                local originalSelection=sim.getObjectSelection()
                for i=2,maxTraces,1 do
                    sim.removeObjectFromSelection(sim.handle_all,-1)
                    sim.addObjectToSelection(sim.handle_single,h)
                    simCopyPasteSelectedObjects()
                    local h2=simGetObjectLastSelection()
                    sim.setObjectParent(h2,model,true)
                    sim.setObjectInt32Parameter(h2,sim.objintparam_visibility_layer,0)
                    table.insert(copiedShapeHandles,h2)
                end
                sim.removeObjectFromSelection(sim.handle_all,-1)
                sim.addObjectToSelection(originalSelection)
            end
            transformationCorrection=mTransf
            previousMatrix=m
            nextIndex=nextIndex+1
            if (nextIndex>maxTraces) then
                nextIndex=1
            end
        else
            originalShapeHandle=-1
        end
    end
end
------------------------------------------------------------------------------ 
-- Following few lines automatically added by V-REP to guarantee compatibility 
-- with V-REP 3.1.3 and earlier: 
colorCorrectionFunction=function(_aShapeHandle_) 
    local version=sim.getInt32Parameter(sim.intparam_program_version) 
    local revision=sim.getInt32Parameter(sim.intparam_program_revision) 
    if (version<30104)and(revision<3) then 
        return _aShapeHandle_ 
    end 
    return '@backCompatibility1:'.._aShapeHandle_ 
end 
------------------------------------------------------------------------------ 
 
 

function sysCall_cleanup() 

end 

function sysCall_actuation() 
    frame=frame+1
    if (originalShapeHandle~=-1) then
        local mNew=sim.getObjectMatrix(originalShapeHandle,-1)
        if (mNew) then
            local dx={previousMatrix[4]-mNew[4],previousMatrix[8]-mNew[8],previousMatrix[12]-mNew[12]}
            local l=math.sqrt(dx[1]*dx[1]+dx[2]*dx[2]+dx[3]*dx[3])
            local axis,angle=sim.getRotationAxis(mNew,previousMatrix)
            if (((l>minPositionalVariation)or(math.abs(angle)>minAngularVariation))and(forceEveryXFrameRecording==0))or(frame==forceEveryXFrameRecording) then
                -- We need to add a new trace at this position
                local h=copiedShapeHandles[nextIndex]
                local m=sim.multiplyMatrices(mNew,transformationCorrection)
                sim.setObjectMatrix(h,-1,m)
                sim.setObjectInt32Parameter(h,sim.objintparam_visibility_layer,initialVisibilityLayers)
                previousMatrix=mNew
                nextIndex=nextIndex+1
                if (nextIndex>maxTraces) then
                    nextIndex=1
                end
                frame=0
            end
        end
    end
end 
