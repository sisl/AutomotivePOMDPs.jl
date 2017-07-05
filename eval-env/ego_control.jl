### Control the ego car in the occluded_crosswalk environment ###

function AutomotiveDrivingModels.propagate(veh::Vehicle, action::Float64, roadway::Roadway, Δt::Float64)
    x_ = veh.state.posG.x + veh.state.v*Δt + action*Δt^2/2
    v_ = veh.state.v + action*Δt
    return VehicleState(VecSE2(x_, veh.state.posG.y, veh.state.posG.θ), roadway, v_)
end
# function AutomotiveDrivingModels.propagate(veh::Vehicle, action::Float64, roadway::Roadway, Δt::Float64)
#     lane_tag_orig = veh.state.posF.roadind.tag
#     state = propagate(veh, LatLonAccel(0., action), roadway, Δt)
#     roadproj = proj(state.posG, roadway[lane_tag_orig], roadway, move_along_curves=false)
#     retval = VehicleState(Frenet(roadproj, roadway), roadway, state.v)
#     return retval
# end

"""
    The driver model to be evaluated, contains the policy and the sensor model. The action is an
    acceleration updated by the observe! function at each time step
"""
type CrosswalkDriver{B, P, U} <: DriverModel{Float64}
    env::CrosswalkEnv
    a::Float64 # acceleration
    b::B # belief
    ego::VehicleState
    sensor::SimpleSensor
    policy::P # replace by policy type
    updater::U
end

AutomotiveDrivingModels.get_name(model::CrosswalkDriver) = "CrosswalkDriver"
Base.rand(model::CrosswalkDriver) = model.a # deterministic policy

function AutomotiveDrivingModels.observe!(model::CrosswalkDriver,
                                          scene::EntityFrame{VehicleState, VehicleDef, Int64},
                                          roadway::Roadway,
                                          egoid::Int)
    @assert scene[1].id == egoid
    model.ego = scene[1].state
    observed_cars = measure(model.ego, scene.entities[2:scene.n], model.sensor, model.env)
    obs = insert!(observed_cars, 1, scene[1])
    # TODO  handle asynchronism
    model.b = update(model.updater, model.b, model.a, obs)# perform belief update
    model.a = action(model.policy, model.b) # policy
end

"""
    initial_ego(env::CrosswalkEnv, rng::AbstractRNG)
generate an initial state for the ego car, returns a Vehicle object
"""
function initial_ego(env::CrosswalkEnv, rng::AbstractRNG)
    x0 = env.params.start_pos
    y0 = 0. # parameterize
    v0 = rand(rng, Uniform(6., env.params.speed_limit))

    return Vehicle(VehicleState(VecSE2(x0, y0, 0.0), env.roadway.segments[1].lanes[1], env.roadway, v0),
                   VehicleDef(), 1)
end
