## Simulation function
# dispatch the simulate method to be problem specific


function _run_callbacks{S,D,I,M<:DriverModel,C<:Tuple{Vararg{Any}}}(callbacks::C, rec::EntityQueueRecord{S,D,I}, env::CrosswalkEnv, models::Dict{I,M}, tick::Int)
    isdone = false
    for callback in callbacks
        isdone |= run_callback(callback, rec, env, models, tick)
    end
    return isdone
end



# add a method that spawn new entities
function AutomotiveDrivingModels.simulate!{S,D,I,A,M<:AutomotiveDrivingModels.DriverModel, C<:Tuple{Vararg{Any}}}(
    ::Type{A},
    rec::EntityQueueRecord{S, D, I},
    scene::EntityFrame{S, D, I},
    env::CrosswalkEnv,
    models::Dict{I, M},
    nticks::Int,
    rng::AbstractRNG,
    callbacks::C
    )

    empty!(rec)
    update!(rec, scene)

    # potential early out right off the bat
    if _run_callbacks(callbacks, rec, env, models, 0)
        return rec
    end

    actions = Array(A, length(scene))

    next_time = -log(rand(rng))/env.params.ped_rate
    time_precision = Int(abs(log10(1/rec.timestep)))
    next_time = round(next_time, time_precision)
    # println("First pedestrian Spawning time: $next_time")
    for tick in 1 : nticks
        time = tick*rec.timestep
        # println("time: $time")
        if time ≈ next_time
            # println("Spawning new pedestrians")
            next_time = time + max(-log(rand(rng))/env.params.ped_rate, rec.timestep)
            next_time = round(next_time, time_precision)
            # println("Next pedestrian at $(next_time)")
            new_ped = initial_pedestrian(env, rng)
            models[new_ped.id] = ConstantPedestrian(0.1, 0.5, 2.0, config.sim_dt)#TODO parameterized
            push!(scene, new_ped)
            actions = Array(A, length(scene))
        end
        get_actions!(actions, scene, env.roadway, models)
        tick!(scene, env.roadway, actions, rec.timestep)
        update!(rec, scene)

        if _run_callbacks(callbacks, rec, env, models, tick)
            break
        end
    end
    return rec
end

function AutomotiveDrivingModels.simulate!{S,D,I,M<:AutomotiveDrivingModels.DriverModel, C<:Tuple{Vararg{Any}}}(
    rec::EntityQueueRecord{S, D, I},
    scene::EntityFrame{S, D, I},
    env::CrosswalkEnv,
    models::Dict{I, M},
    nticks::Int,
    rng::AbstractRNG,
    callbacks::C
    )

    return simulate!(Any, rec, scene, env, models, nticks, rng, callbacks)
end
