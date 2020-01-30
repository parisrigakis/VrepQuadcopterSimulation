function sysCall_init() 
    -- Make sure we have version 2.4.13 or above (the particles are not supported otherwise)
    v=sim.getInt32Parameter(sim.intparam_program_version)

    if (v<20413) then
        sim.displayDialog('Warning','The propeller model is only fully supported from V-REP version 2.4.13 and above.&&nThis simulation will not run as expected!',sim.dlgstyle_ok,false,'',nil,{0.8,0,0,0,0,0})
    end

    -- Detatch the manipulation sphere:
    targetObj=sim.getObjectHandle('Quadricopter_target')
    
    sim.setObjectParent(targetObj,-1,true)
    usensors1 = sim.getObjectHandle("Ultsens1")
    usensors2 = sim.getObjectHandle("Ultsens2")
    usensors3 = sim.getObjectHandle("Ultsens3")
    usensors4 = sim.getObjectHandle("Ultsens4")
    potforce1 = 0
    potforce2 = 0
    potforce3 = 0
    potforce4 = 0
    dinami = {}
    -- This control algo was quickly written and is dirty and not optimal. It just serves as a SIMPLE example

    d=sim.getObjectHandle('Quadricopter_base')

    particlesAreVisible=sim.getScriptSimulationParameter(sim.handle_self,'particlesAreVisible')
    sim.setScriptSimulationParameter(sim.handle_tree,'particlesAreVisible',tostring(particlesAreVisible))
    simulateParticles=sim.getScriptSimulationParameter(sim.handle_self,'simulateParticles')
    sim.setScriptSimulationParameter(sim.handle_tree,'simulateParticles',tostring(simulateParticles))

    propellerScripts={-1,-1,-1,-1}
    for i=1,4,1 do
        propellerScripts[i]=sim.getScriptHandle('Quadricopter_propeller_respondable'..i)
    end
    heli=sim.getObjectAssociatedWithScript(sim.handle_self)

    particlesTargetVelocities={0,0,0,0}

    pParam=2
    iParam=0
    dParam=0
    vParam=-2

    cumul=0
    lastE=0
    pAlphaE=0
    pBetaE=0
    psp2=0
    psp1=0

    prevEuler=0


    fakeShadow=sim.getScriptSimulationParameter(sim.handle_self,'fakeShadow')
    if (fakeShadow) then
        shadowCont=sim.addDrawingObject(sim.drawing_discpoints+sim.drawing_cyclic+sim.drawing_25percenttransparency+sim.drawing_50percenttransparency+sim.drawing_itemsizes,0.2,0,-1,1)
    end

    -- Prepare 2 floating views with the camera views:
    floorCam=sim.getObjectHandle('Quadricopter_floorCamera')
    frontCam=sim.getObjectHandle('Quadricopter_frontCamera')
    floorView=sim.floatingViewAdd(0.9,0.9,0.2,0.2,0)
    frontView=sim.floatingViewAdd(0.7,0.9,0.2,0.2,0)
    sim.adjustView(floorView,floorCam,64)
    sim.adjustView(frontView,frontCam,64)
end

function sysCall_cleanup() 
    sim.removeDrawingObject(shadowCont)
    sim.floatingViewRemove(floorView)
    sim.floatingViewRemove(frontView)
end 

function sysCall_actuation() 
    s=sim.getObjectSizeFactor(d)
    
    pos=sim.getObjectPosition(d,-1)
    if (fakeShadow) then
        itemData={pos[1],pos[2],0.002,0,0,1,0.2*s}
        sim.addDrawingObjectItem(shadowCont,itemData)
    end
    
    -- Vertical control:
    targetPos=sim.getObjectPosition(targetObj,-1)
    pos=sim.getObjectPosition(d,-1)
    l=sim.getVelocity(heli)
    e=(targetPos[3]-pos[3])
    cumul=cumul+e
    pv=pParam*e
    thrust=5.335+pv+iParam*cumul+dParam*(e-lastE)+l[3]*vParam
    lastE=e
    
    -- Horizontal control: 
    sp=sim.getObjectPosition(targetObj,d)
    m=sim.getObjectMatrix(d,-1)
    vx={1,0,0}
    vx=sim.multiplyVector(m,vx)
    vy={0,1,0}
    vy=sim.multiplyVector(m,vy)
    alphaE=(vy[3]-m[12])
    alphaCorr=0.25*alphaE+2.1*(alphaE-pAlphaE)
    betaE=(vx[3]-m[12])
    betaCorr=-0.25*betaE-2.1*(betaE-pBetaE)
    pAlphaE=alphaE
    pBetaE=betaE
    alphaCorr=alphaCorr+sp[2]*0.005+1*(sp[2]-psp2)
    betaCorr=betaCorr-sp[1]*0.005-1*(sp[1]-psp1)
    psp2=sp[2]
    psp1=sp[1]
    
    -- Rotational control:
    euler=sim.getObjectOrientation(d,targetObj)
    rotCorr=euler[3]*0.1+2*(euler[3]-prevEuler)
    prevEuler=euler[3]
    --Keyboard Control
    
    message,auxiliaryData=sim.getSimulatorMessage()
    while message~=-1 do
        if (message==sim.message_keypress) then
            if (auxiliaryData[1]==2007) then
                -- up key
                targetPos[1] = targetPos[1] + 0.01
            end
           
        
            if (auxiliaryData[1]==2008) then
                -- down key
                targetPos[1] = targetPos[1] - 0.01
            end
           
        
            if (auxiliaryData[1]==2009) then
                -- left key
                targetPos[2] = targetPos[2] + 0.01
            end
           
        
            if (auxiliaryData[1]==2010) then
                -- right key
                targetPos[2] = targetPos[2] - 0.01
            end
            if (auxiliaryData[1]==119) then
                -- right key
                targetPos[3] = targetPos[3] + 0.01
            end
            if (auxiliaryData[1]==115) then
                -- right key
                targetPos[3] = targetPos[3] - 0.01
            end

        end
        message,auxiliaryData=sim.getSimulatorMessage()   
        
        
    end
    potforce1 = 0
    potforce2 = 0
    potforce3 = 0
    potforce4 = 0

    res1,dist1=sim.readProximitySensor(usensors1)
    res2,dist2=sim.readProximitySensor(usensors2)
    res3,dist3=sim.readProximitySensor(usensors3)
    res4,dist4=sim.readProximitySensor(usensors4)
    if (res1>0)    then
        if dist1 < 0.5   then
            potforce1 = (1/0.7 - 1/dist1)*0.005
        end
    end
    if (res2>0)    then
        if dist2 < 0.5   then
            potforce2 = (1/0.7 - 1/dist2)*0.005
        end
    end
    if (res3>0)    then
        if dist3 < 0.5   then
            potforce3 = (1/0.7 - 1/dist3)*0.005
        end
    end
    if (res4>0)    then
        if dist4 < 0.5   then
            potforce4 = (1/0.7 - 1/dist4)*0.005
        end
    end
    targetPos[1] = targetPos[1] + potforce1
    targetPos[1] = targetPos[1] - potforce2
    targetPos[2] = targetPos[2] - potforce3
    targetPos[2] = targetPos[2] + potforce4
    normebod = ((potforce1 - potforce2)^2 + (potforce4 -potforce3)^2)^0.5
    dinami[1] = 5 - pos[1]
    dinami[2] = -pos[2]
    normdin = (dinami[1]^2 + dinami[2]^2)^0.5
    targetPos[1] = targetPos[1] + (dinami[1]/normdin)*0.005
    targetPos[2] = targetPos[2] + (dinami[2]/normdin)*0.005
    --if math.abs(0.005 - normebod) < 0.002 then
    --    if res1>0 and   res3 == 0 and res4 == 0  then
    --        targetPos[2] = targetPos[2] + 0.01
    --    elseif res1>0 and res3 >0 and res4 == 0  then
    --        targetPos[2] = targetPos[2] + 0.01
    --        
    --    elseif res1 > 0 and res3 == 0 and res4 > 0  then
    --        targetPos[2] = targetPos[2] + 0.005
    --        targetPos[1] = targetPos[1] - 0.005
    --    elseif res1 > 0 and res3 > 0 and res4 > 0 then
    --        targetPos[2] = targetPos[2] + 0.01
    --    elseif res1 > 0 and res3 == 0 and res4 > 0  then
    --        targetPos[2] = targetPos[2] + 0.005
    --        targetPos[1] = targetPos[1] - 0.005
    --    elseif res1 == 0 and res3 > 0 and res4 == 0  then
    --        targetPos[2] = targetPos[2] + 0.005
    --        targetPos[1] = targetPos[1] - 0.005
    --    end 
        
            
    --end
            
    
    sim.setObjectPosition(targetObj,-1,targetPos)
    sim.addStatusbarMessage(0.005 - normebod)
    -- Decide of the motor velocities:
    particlesTargetVelocities[1]=thrust*(1-alphaCorr+betaCorr+rotCorr)
    particlesTargetVelocities[2]=thrust*(1-alphaCorr-betaCorr-rotCorr)
    particlesTargetVelocities[3]=thrust*(1+alphaCorr-betaCorr+rotCorr)
    particlesTargetVelocities[4]=thrust*(1+alphaCorr+betaCorr-rotCorr)
    
    -- Send the desired motor velocities to the 4 rotors:
    for i=1,4,1 do
        sim.setScriptSimulationParameter(propellerScripts[i],'particleVelocity',particlesTargetVelocities[i])
    end
end 
