### Control the ego car in the occluded_crosswalk environment ###
# Each policy should implement its own version of propagate corresponding to the returned action type
function AutomotiveDrivingModels.propagate(veh::Vehicle, action::Float64, roadway::Roadway, Δt::Float64)
    x_ = veh.state.posG.x + veh.state.v*Δt + action*Δt^2/2
    if x_ <= veh.state.posG.x
        x_ = veh.state.posG.x
    end
    v_ = veh.state.v + action*Δt
    if v_ <= 0.
        v_ = 0.
    elseif v_ >= 8.0 #TODO parameterized
        v_ = 8.0
    end
    return VehicleState(VecSE2(x_, veh.state.posG.y, veh.state.posG.θ), roadway, v_)
end

"""
    The driver model to be evaluated, contains the policy and the sensor model. The action is an
    acceleration updated by the observe! function at each time step
"""
mutable struct AVDriver{A, B, O, P, U, S} <: DriverModel{Float64}
    update_freq::Int64
    tick::Int64
    env::CrosswalkEnv
    a::A# acceleration
    b::B # belief
    o::O
    ego::Vehicle
    sensor::S
    policy::P # replace by policy type
    updater::U
end

AutomotiveDrivingModels.get_name(model::AVDriver) = "AVDriver"
Base.rand(model::AVDriver) = model.a # deterministic policy

function AutomotiveDrivingModels.observe!(model::AVDriver,
                                          scene::EntityFrame{VehicleState, VehicleDef, Int64},
                                          roadway::Roadway,
                                          egoid::Int)
    # @assert scene[1].id == egoid
    model.o = measure(scene, model.sensor, env)
    b_ = update(model.updater, model.b, model.a, model.o)
    # f = open("timing.csv", "a")
    # write(f,"belief, $btime, $(scene.n - 1) \n")
    # close(f)
    model.b = deepcopy(b_)
    if model.tick % model.update_freq == 0
        model.a = action(model.policy, model.b) # policy
        # f = open("timing.csv", "a")
        # write(f, "action, $atime, $(scene.n - 1) \n")
        # close(f)
    end
    model.tick += 1
end

"""
    initial_ego(env::CrosswalkEnv, rng::AbstractRNG)
generate an initial state for the ego car, returns a Vehicle object
"""
function initial_ego(env::CrosswalkEnv, rng::AbstractRNG)
    x0 = env.params.start_pos
    y0 = 0. # parameterize
    v0 = rand(rng, Uniform(6., env.params.speed_limit))
    # v0 = 8.

    return Vehicle(VehicleState(VecSE2(x0, y0, 0.0), env.roadway.segments[1].lanes[1], env.roadway, v0),
                   VehicleDef(), 1)
end

function reset_model!(model::AVDriver, ego::Vehicle)
    model.tick = 0.
    model.a = 0.
    model.o = Dict{Int64, SingleOCState}()
    # model.b = initial_state_distribution(model.policy.pomdp, model.sensor.n_ped)
    b0 = Dict{Int64, SingleOCDistribution}()
    b0[-1] = initial_state_distribution(policy.pomdp, ego.state)
    model.b = b0
    model.ego = ego
    reset_policy!(model.policy)
end
