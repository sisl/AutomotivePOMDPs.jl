## Simulation function
# dispatch the simulate method to be problem specific

function AutomotiveDrivingModels.simulate!{S,D,I,A,M<:AutomotiveDrivingModels.DriverModel, C<:Tuple{Vararg{Any}}}(
    ::Type{A},
    rec::EntityQueueRecord{S, D, I},
    scene::EntityFrame{S, D, I},
    env::CrosswalkEnv,
    models::Dict{I, M},
    nticks::Int,
    rng::AbstractRNG,
    n_ped::Int64,
    callbacks::C
    )

    empty!(rec)
    update!(rec, scene)

    # potential early out right off the bat
    if _run_callbacks(callbacks, rec, env, models, 0)
        return rec
    end

    actions = Array{A}(length(scene))

    # next_time = -log(rand(rng))/env.params.ped_rate
    # time_precision = Int(abs(log10(1/rec.timestep)))
    # next_time = round(next_time, time_precision)
    # println("First pedestrian Spawning time: $next_time")
    for tick in 1 : nticks
        # time = tick*rec.timestep
        # # println("time: $time")
        # if time ≈ next_time && scene.n < n_ped + 1
        #     # println("Spawning new pedestrians")
        #     next_time = time + max(-log(rand(rng))/env.params.ped_rate, rec.timestep)
        #     next_time = round(next_time, time_precision)
        #     # println("Next pedestrian at $(next_time)")
        #     new_ped = initial_pedestrian(scene, env, rng)
        #     models[new_ped.id] = ConstantPedestrian(dt = config.sim_dt)#TODO parameterized
        #     push!(scene, new_ped)
        #     actions = Array{A}(length(scene))
        # end
        if rand(config.rng) < env.params.ped_rate
            # println("Spawning new pedestrians")
            new_ped = initial_pedestrian(scene, env, config.rng)
            models[new_ped.id] = ConstantPedestrian(dt = config.sim_dt)#TODO parameterized
            push!(scene, new_ped)
            actions = Array{Any}(length(scene))
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
    n_ped::Int64,
    callbacks::C
    )

    return simulate!(Any, rec, scene, env, models, nticks, rng, n_ped, callbacks)
end

function _run_callbacks{S,D,I,M<:DriverModel,C<:Tuple{Vararg{Any}}}(callbacks::C, rec::EntityQueueRecord{S,D,I}, env::CrosswalkEnv, models::Dict{I,M}, tick::Int)
    isdone = false
    for callback in callbacks
        isdone |= run_callback(callback, rec, env, models, tick)
    end
    return isdone
end

"""
    initial_pedestrian(scene::Scene, env::CrosswalkEnv, scene::Scene, rng::AbstractRNG)
Create a new pedestrian entity at its initial state
"""
function initial_pedestrian(scene::Scene, env::CrosswalkEnv, rng::AbstractRNG)
    # lateral position
    d_lat = Uniform(env.params.roadway_length - env.params.crosswalk_width + 2, env.params.roadway_length + env.params.crosswalk_width-1)
    x0 = 0.5*rand(rng, d_lat)
    # x0 =  0.5*env.params.roadway_length

    # vertical position
    y0 = -env.params.crosswalk_length/4

    # orientation
    θ = π/2

    #velocity
    v0 = rand(rng, Uniform(0., env.params.ped_max_speed))
    cw_roadway = Roadway([RoadSegment(2, [env.crosswalk])]);
    ped_initial_state = VehicleState(VecSE2(x0, y0, θ), env.crosswalk, cw_roadway, v0);
    # ped_initial_state = VehicleState(VecSE2(x0, y0, θ), env.crosswalk, env.roadway, v0)

    # new id, increment last id
    id = scene.n + 2

    return Vehicle(ped_initial_state,  VehicleDef(AgentClass.PEDESTRIAN, 1.0, 1.0), id)
end



function initial_scene(models::Dict{Int64, DriverModel}, env::CrosswalkEnv, config::EvalConfig)
    scene = Scene()
    n_ped = rand(config.rng, 0:10)
    n_ped = 0
    ego = models[1].ego.state
    pomdp = models[1].updater.problem
    b0 = initial_state_distribution(pomdp, ego)
    for i = 1:n_ped
        ped_state = rand(config.rng, b0).ped
        d_lat = Uniform(env.params.roadway_length - env.params.crosswalk_width + 2, env.params.roadway_length + env.params.crosswalk_width-1)
        x0 = 0.5*rand(config.rng, d_lat)
        y0 = ped_state.posG.y
        cw_roadway = Roadway([RoadSegment(2, [env.crosswalk])])
        ped_state = VehicleState(VecSE2(x0, y0, ped_state.posG.θ), env.crosswalk, cw_roadway, ped_state.v)

        if !off_the_grid(pomdp, ped_state)
            id = scene.n + 2
            new_ped = Vehicle(ped_state, PEDESTRIAN_DEF, id)
            models[new_ped.id] = ConstantPedestrian(dt = config.sim_dt)
            push!(scene, new_ped)
        end
    end
    # println("Starting with ", scene.n, " pedestrians on the scene")
    return scene
end


# function initial_scene(models::Dict{Int64, DriverModel}, env::CrosswalkEnv,
#                        config::EvalConfig)
#
#     nticks = rand(config.rng, 0:200)
#     rec = SceneRecord(nticks+1, config.sim_dt)
#     scene = Scene()
#     empty!(rec)
#     update!(rec, scene)
#     actions = Array{Any}(length(scene))
#
#     for tick in 1 : nticks
#         time = tick*rec.timestep
#         # println("time: $time")
#         if rand(config.rng) < env.params.ped_rate
#             # println("Spawning new pedestrians")
#             new_ped = initial_pedestrian(scene, env, config.rng)
#             models[new_ped.id] = ConstantPedestrian(dt = config.sim_dt)#TODO parameterized
#             push!(scene, new_ped)
#             actions = Array{Any}(length(scene))
#         end
#
#         get_actions!(actions, scene, env.roadway, models)
#         tick!(scene, env.roadway, actions, rec.timestep)
#         update!(rec, scene)
#     end
#     @assert findfirst(EGO_ID, scene) == 0
#     return rec[0]
# end
