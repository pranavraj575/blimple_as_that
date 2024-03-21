require 'blimpHeader'
require 'rosMsg'
require 'common'

-- Function to calculate and return the lift force provided by the envelope to help offset gravity
function calculateInherentLiftForce()
    return {0,0,MASS_BLIMP_EST*-GRAVITY}
end

-- Function to calculate and return an estimated drag force
function calculateEstimatedDragForce(blimpHandle)
    local linVel, angVel = sim.getVelocity(blimpHandle)

    local abs2DVel = math.sqrt(linVel[1]*linVel[1] + linVel[2]*linVel[2])
    local theta = math.atan2(linVel[2], linVel[1])

    -- local airDensity = 1.22 --kg/(m^3)
    -- local crossArea = 0.19949
    -- local coefficient = 0.4
    -- local forceDrag = 0.5 * airDensity * abs2DVel * abs2DVel * coefficient * crossArea
    -- no longer need these coefficients as we can calculate the coefficient based on real world data

    local forceDrag = DRAG_FORCE_COEFFICIENT * abs2DVel * abs2DVel
    local xForce = -1 * forceDrag * math.cos(theta)
    local yForce = -1 * forceDrag * math.sin(theta)
    -- print(string.format("vel: %.4f | fxdrag:%.3f | y drag:%.3f", abs2DVel, xForce, yForce))
    return {xForce,yForce,0}
end

-- Function to return the id number based on the model name that adheres to
-- Coppeliasim's automatic naming convention.
-- Models without a '#' are typically the first/original model and will have id 0
-- Models with a '#' and then a num, start incrementing from 0,
-- so we add 1 to its number to offset it from the first/original model
function getIdFromName(nameStr)
    local result = string.find(nameStr, '#')
    if result then
        local hashSubstr = string.sub(nameStr, result + 1) -- +1 to ignore the hash
        return (tonumber(hashSubstr) + 1) -- +1 to offset from base model name not having a hash
    else
        return 0
    end
end

function addGlobalFlowForce(blimpHandle)

    if blimpState.position.z < 2 then
         return {0.005,0,0}
    else
        return {-0.005,0,0}

    end
end

function createStateTable()
    blimpState = {ultrasonic=0,
                position={x=0,y=0,z=0},
                prev_position={x=0,y=0,z=0},
                orientation={x=0,y=0,z=0},
                prev_orientation={x=0,y=0,z=0},
                velocity = {x=0,y=0,z=0},
                angular_velocity = {x=0,y=0,z=0},
                prevHeightError = 0,
                prevOmegaError = 0,
                prevDistError = 0}
    return blimpState
end

function ctrlsFromInput(vec)
    return ctrlsFromVel({vec[1],vec[2],vec[3]})
end

function ctrlsFromInput2(vec)
    -- print(vec)
    return ctrlsFromVelocity2(vec)
end

function ctrlsFromVelocity2(target_vel)


    -- need to update this for local and global frame, if we want heading control
    MEM=0.
    blimpPosition = blimpState["position"]
    blimpOrientation = blimpState["orientation"]
    old_position=blimpState["prev_position"]
    old_orientation=blimpState["prev_orientation"]
    xp,yp,zp=blimpPosition["x"],blimpPosition["y"], blimpPosition["z"]

    heading = blimpOrientation["z"]  --radians

    -- print("heading", blimpOrientation["z"])

    -- COnvert to Local Frame
    vel_x=(xp-old_position["x"])/SIM_DT * math.cos(heading)
    vel_y=(yp-old_position["y"])/SIM_DT * math.sin(heading)
    vel_z=(zp-old_position["z"])/SIM_DT

    -- vel_x = (xp-old_position["x"]*math.cos(heading) + (yp-old_position["y"]*math.cos(heading)

    blimpState["prev_position"]["x"]=(1-MEM)*xp+MEM*old_position["x"]
    blimpState["prev_position"]["y"]=(1-MEM)*yp+MEM*old_position["y"]
    blimpState["prev_position"]["z"]=(1-MEM)*zp+MEM*old_position["z"]

     --print("vel_x", Round(vel_x,2), "vel_y", Round(vel_y,2), "vel_z", Round(vel_z,2))

    vel_z_theta=(blimpOrientation["z"]-old_orientation["z"])/SIM_DT

    blimpState["prev_orientation"]["z"]=(1-MEM)*blimpOrientation["z"]+MEM*old_orientation["z"]


    -- This if/else pair eliminates hovering
    if target_vel[3] > 0 and vel_z > MAX_ALT_VEL then
        bMotorForce = 0
    elseif target_vel[3] < 0 and vel_z < -MAX_ALT_VEL then
        bMotorForce = 0
    else
        bMotorForce = target_vel[3]
    end

    return ctrlsFromForce2({target_vel[1],target_vel[2],bMotorForce, 0, 0, target_vel[6]})
end

function ctrlsFromForce2(target_force)
    local lMotorForce, rMotorForce, bMotorForce = 0,0,0

    -- print(target_force)

    lMotorForce, rMotorForce, bMotorForce = 0,0,0

    theta = target_force[6]

    rMotorForce = target_force[1] + target_force[6]
    lMotorForce = target_force[1] - target_force[6]

    --print(Round(rMotorForce,2), Round(lMotorForce,2), Round(target_force[1],2), Round(target_force[6],2))

    rMotorForce = clamp(rMotorForce, RMOTOR_MAX_FORCE, -RMOTOR_MAX_FORCE)
    lMotorForce = clamp(lMotorForce, LMOTOR_MAX_FORCE, -LMOTOR_MAX_FORCE)
    bMotorForce = clamp(target_force[3], BMOTOR_MAX_FORCE, -BMOTOR_MAX_FORCE)

    return lMotorForce, rMotorForce, bMotorForce
end



-- Retrieve the global position/orientation and format it as a twistStamped msg
function getStateGlobal(robotId, objHandle, relTo)
    t = simROS2.getSimulationTime() -- simROS2.getSystemTime()
    p = sim.getObjectPosition(objHandle, relTo)
    o = sim.getObjectOrientation(objHandle, relTo)

    return {
        header = {
            stamp = t,
            frame_id = tostring(robotId),
        },
        twist = {
            linear = { x = p[1], y = p[2], z = p[3]},
            angular = { x = o[1], y = o[2], z = o[3]}
        }
    }
end

-- Retrieve the proximity sensors reading and format as a float msg
-- Done for special handling of how proximity sensor returns are returned by Coppeliasim
function getStateProximity(proximityHandle)
    local result, distance = sim.readProximitySensor(proximityHandle)
    if result then
        if result == 0 then
            -- no object detected, means open space, return the maximum detectable distance
            return { data = MAX_PROXIMITY_DIST }
        elseif result == 1 then
            -- object detected, so we have a distance to report
            if distance < MIN_PROXIMITY_DIST then
                distance = MIN_PROXIMITY_DIST
            end
            local gauss = gaussian(0,ULTRASONIC_NOISE_VARIANCE)
            -- local gauss = 0
            -- print(string.format("%.4f",gauss))
            return { data = distance + gauss }
        end
    end

    -- There was either no result (maybe bad handle) or an error from the sensor
    return { data = -1 }
end
