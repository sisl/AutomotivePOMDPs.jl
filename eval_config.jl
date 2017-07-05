#### DEFINE THE EVALUATION PARAMETERS ################

"""
Container type for the evaluation script parameters
#Fields
- `sim_dt` simulation time step
"""
type EvalConfig
    sim_dt::Float64
    rng::AbstractRNG
    n_episodes::Int64
    time_out::Int64
    callbacks::Tuple{Vararg{Any}}
    #verbose
    #render
end

type TerminalCallback end

type EgoCollisionCallback end

function EvalConfig(; sim_dt::Float64 = 0.1,
                      rng::AbstractRNG = MersenneTwister(3),
                      n_episodes::Int64 = 1,
                      time_out::Int64 = 200,
                      callbacks::Tuple{Vararg{Any}} = (EgoCollisionCallback(),TerminalCallback())
                      )
    return EvalConfig(sim_dt, rng, n_episodes, time_out, callbacks)
end

## Define Callbacks

function AutomotiveDrivingModels.run_callback{S,D,I,M<:DriverModel}(
    callback::EgoCollisionCallback,
    rec::EntityQueueRecord{S,D,I},
    env::CrosswalkEnv,
    models::Dict{I,M},
    tick::Int,
    )
    return is_crash(rec[0])
end

function is_crash(scene::Scene)
    ego = scene.entities[1]
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
    ego = scene.entities[1]
    @assert ego.id == 1
    return ego.state.posG.x > env.params.end_pos
end
