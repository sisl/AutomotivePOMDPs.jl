#### DEFINE THE EVALUATION PARAMETERS ################

"""
Container type for the evaluation script parameters
#Fields
- `sim_dt` simulation time step
- rng::AbstractRNG
- n_episodes::Int64
- time_out::Int64
- callbacks::Tuple{Vararg{Any}}
"""
struct EvalConfig
    sim_dt::Float64
    rng::AbstractRNG
    n_episodes::Int64
    time_out::Int64
    callbacks::Tuple{Vararg{Any}}
    n_ped::Int64
    #verbose
    #render
end

function EvalConfig(; sim_dt::Float64 = 0.1,
                      rng::AbstractRNG = MersenneTwister(3),
                      n_episodes::Int64 = 1,
                      time_out::Int64 = 200,
                      callbacks::Tuple{Vararg{Any}} = (EgoCollisionCallback(),TerminalCallback()),
                      n_ped::Int64 = 2
                      )
    return EvalConfig(sim_dt, rng, n_episodes, time_out, callbacks, n_ped)
end

function Base.show(io::IO, conf::EvalConfig)
    @printf(io, "EvalConfig \n")
    @printf(io, "\t Sim step (s) %.2f \n", conf.sim_dt)
    @printf(io, "\t N episodes %4i \n", conf.n_episodes)
    @printf(io, "\t Time out %4i \n", conf.time_out)
    @printf(io, "\t Time out %4i \n", conf.time_out)
end


## Define Callbacks

struct TerminalCallback end

struct EgoCollisionCallback end

function AutomotiveDrivingModels.run_callback{S,D,I,M<:DriverModel}(
    callback::EgoCollisionCallback,
    rec::EntityQueueRecord{S,D,I},
    env::CrosswalkEnv,
    models::Dict{I,M},
    tick::Int,
    )
    return is_crash(rec[0])
end

"""
    is_crash(scene::Scene)
return true if the ego car is in collision in the given scene, do not check for collisions between
other participants
"""
function is_crash(scene::Scene)
    ego = scene[findfirst(isequal(1), scene)]
    @assert ego.id == 1
    if ego.state.v ≈ 0
        return false
    end
    for veh in scene
        if veh.id != 1
            if is_colliding(ego, veh)
                return true
            end
        end
    end
    return false
end

function AutomotiveDrivingModels.run_callback{S,D,I,M<:DriverModel}(
    callback::TerminalCallback,
    rec::EntityQueueRecord{S,D,I},
    env::CrosswalkEnv,
    models::Dict{I,M},
    tick::Int,
    )

    return is_terminal(rec[0], env)
end

function is_terminal(scene::Scene, env::CrosswalkEnv)
    # check ego car position
    ego = scene[findfirst(isequal(1), scene)]
    @assert ego.id == 1
    return ego.state.posG.x > env.params.end_pos
end
