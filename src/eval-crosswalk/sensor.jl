


"""
Sensor model with gaussian noise in position and velocity measurement
"""
type SimpleSensor
    pos_noise::Float64
    vel_noise::Float64
end

@with_kw mutable struct POMDPSensor
    pomdp::SingleOCPOMDP = SingleOCPOMDP()
    sensor::SimpleSensor = SimpleSensor(0., 0.)
end

const OFF_KEY = -1

"""
    measure(scene::Scene, sensor::POMDPSensor, env::CrosswalkEnv)
return a measurement of a given scene from the point of view of the ego car
"""
function measure(scene::Scene, sensor::POMDPSensor, env::CrosswalkEnv)
    pos_noise = sensor.sensor.pos_noise
    vel_noise = sensor.sensor.vel_noise
    pomdp = sensor.pomdp
    o = Dict{Int64, SingleOCObs}()
    ego = scene[findfirst(scene, 1)].state
    for i=1:scene.n
        veh = scene[i]
        if veh.id == 1
            continue
        end
        car = veh.state
        if is_observable_fixed(ego, car, env)
            y_pos = min(pomdp.y_goal, car.posG.y + pos_noise*randn())
            obs_state = VehicleState(VecSE2(car.posG.x + pos_noise*randn(),
                                                      y_pos,
                                                      car.posG.θ),
                                                      env.roadway,
                                                      car.v + vel_noise*randn())
            collision = collision_checker(ego, veh.state, pomdp.ego_type, veh.def)
            o[veh.id] = SingleOCState(collision, ego, veh.state)
        end
    end
    o[OFF_KEY] = SingleOCState(false, ego, get_off_the_grid(pomdp))
    return o
end

"""
    measure(scene::Scene, sensor::SimpleSensor, env::CrosswalkEnv)
"""
function measure(scene::Scene, sensor::SimpleSensor, env::CrosswalkEnv)
    pos_noise = sensor.pos_noise
    vel_noise = sensor.vel_noise
    o = Vehicle[]
    ego = scene[findfirst(scene, 1)]
    push!(o, ego)
    for veh in scene
        if veh.id == ego.id
            continue
        end
        car = veh.state
        if is_observable_fixed(ego.state, car, env)
            y_pos = min(pomdp.y_goal, car.posG.y + pos_noise*randn())
            obs_state = VehicleState(VecSE2(car.posG.x + pos_noise*randn(),
                                                      y_pos,
                                                      car.posG.θ),
                                                      env.roadway,
                                                      car.v + vel_noise*randn())
            push!(o, Vehicle(obs_state, VehicleDef(), veh.id))
        end
    end
    return o
end

"""
    measure(ego::VehicleState, cars::EntityFrame{VehicleState, VehicleDef, Int64}, model::SimpleSensor, env::CrosswalkEnv)
return a measurement of a given scene from the point of view of the ego car
"""
function measure(ego::Vehicle, cars::Vector{Entity{VehicleState, VehicleDef, Int64}}, model::SimpleSensor, env::CrosswalkEnv)
    observed = Vehicle[] # sizehint!(Vector{Vehicle(0)}, 5)
    for veh in cars
        car = veh.state
        if is_observable_fixed(ego.state, car, env)
            obs_state = VehicleState(VecSE2(car.posG.x + model.pos_noise*randn(),
                                                      car.posG.y + model.pos_noise*randn(),
                                                      car.posG.θ),
                                                      env.roadway,
                                                      car.v + model.vel_noise*randn())
            push!(observed, Vehicle(obs_state, veh.def, veh.id))
        end
    end
    insert!(observed, 1, ego)
    return observed
end

