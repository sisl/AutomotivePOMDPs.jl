# retrieve the parameter to change
collision_cost = float(ARGS[1])
@eval @everywhere collision_cost = $collision_cost

@everywhere begin
    using POMDPs, StatsBase, POMDPToolbox, SARSOP, QMDP, AutomotiveDrivingModels, JLD, Parameters, Distributions, GridInterpolations

    include("../../admenv/crosswalk/occluded_crosswalk_env.jl")
    include("../../admenv/crosswalk/pomdp_types.jl")
    include("../../admenv/crosswalk/spaces.jl")
    include("../../admenv/crosswalk/transition.jl")
    include("../../admenv/crosswalk/observation.jl")
    include("../../admenv/crosswalk/belief.jl")
    include("../../admenv/crosswalk/adm_helpers.jl")
    include("../../admenv/crosswalk/decomposition.jl")

    include("config.jl")
    include("ego_control.jl")
    include("sensor.jl")
    include("policy.jl")
    include("state_estimation.jl")
    include("constant_pedestrian.jl")
    include("simulation.jl")
    include("baseline_policy.jl")

    function evaluate(env::CrosswalkEnv, ego_model::CrosswalkDriver, config::EvalConfig)
        crash = 0
        avg_step = 0
        success = 0
        time_out = 0
        avg_step = 0
        ego0 = initial_ego(env, config.rng)
        reset_model!(ego_model, ego0)
        models = Dict{Int, DriverModel}()
        models[1] = ego_model
        scene = initial_scene(models, env, config)
        push!(scene, ego0)
        nticks = config.time_out
        rec = SceneRecord(nticks+1, config.sim_dt)

        # execute the simulation
        simulate!(rec, scene, env, models, nticks, config.rng, config.n_ped, config.callbacks)
        avg_step = nframes(rec)

        # analyse output
        if is_crash(rec[0])
            crash = 1
            println("Episode CRASH")
        elseif is_terminal(rec[0], env)
            success = 1
            println("Episode SUCCESS")
        elseif nframes(rec) >= config.time_out
            time_out = 1
            println("Episode  TIME OUT")
        else
            println("Episode nothing happened, frames: $(nframes(rec))")
        end

        println("End of the simulation: Crashes $crash , Successes $success, Time out $time_out, Avg Step $(avg_step)")
        return crash, success, time_out, avg_step
    end

    config = EvalConfig(time_out = 300,
                        n_ped=2,
                        n_episodes = 1000)


    params = EnvParams(ped_rate = 0.005)
    env = CrosswalkEnv(params)

    pomdp = OCPOMDP()
    pomdp.collision_cost = collision_cost
    qmdp_policy = load("crosswalk_policy$(abs(pomdp.collision_cost)).jld")
    qmdp_policy = qmdp_policy["qmdp_policy"]

    sensor = POMDPSensor(pomdp = pomdp, sensor = SimpleSensor(0.5, 0.5))
    policy = QMDPEval(env, pomdp, qmdp_policy);
    up = MixedUpdater(pomdp, config.sim_dt);
    a0 = 0.
    o0 = Dict{Int64, OCObs}()
    ego0 = initial_ego(env, config.rng)
    b0 = Dict{Int64, OCDistribution}()
    b0[-1] = initial_state_distribution(policy.pomdp, ego0.state)
    update_freq = 5
    ego_model = CrosswalkDriver(update_freq, 0, env, a0, b0, o0, ego0, sensor, policy, up)
    reset_model!(ego_model, ego0)
end


envs = fill(env, config.n_episodes)
ego_models = fill(ego_model, config.n_episodes)
configs = Vector{EvalConfig}(config.n_episodes)
for i=1:config.n_episodes
    configs[i] = EvalConfig(time_out = config.time_out, n_ped = config.n_ped, n_episodes = config.n_episodes,
                            rng = MersenneTwister(i))
end
evals = pmap(evaluate, envs, ego_models, configs)

crashes = zeros(config.n_episodes)
successes = zeros(config.n_episodes)
time_outs = zeros(config.n_episodes)
steps = zeros(config.n_episodes)
for (i,t) in enumerate(evals)
    crashes[i], successes[i], time_outs[i], steps[i] = t
end

c_rate = mean(crashes)*100
suc_rate = mean(successes)*100
to_rate = mean(time_outs)*100
avg_time = mean(steps)*config.sim_dt

println("Outcome of $(config.n_episodes) episodes : \n",
         "$c_rate \% crashes \n ",
         "$suc_rate\% successes \n ",
         "$to_rate\% time out \n ",
         "$(avg_time)(s) avg time \n ")

scenario = "crosswalk-min"
goal_reward = pomdp.goal_reward
collision_cost = pomdp.collision_cost
p_birth = pomdp.p_birth
n_ped = config.n_ped


result_file = "../results_qmdp.csv"
if !isfile(result_file)
    f = open(result_file, "w")
    write(f, "scenario, goal_reward, action_cost, collision_cost, p_birth, n_ped, n_episodes, crashes, successes, time_out, avg_time \n")
else
    f = open(result_file, "a")
end
res_str = string(scenario, ", ", pomdp.goal_reward, ", ", pomdp.action_cost, ", ", pomdp.collision_cost, ", ", pomdp.p_birth, ", ",
                 config.n_ped, ", ", config.n_episodes, ", ",
                 c_rate, ", ", suc_rate, ", ", to_rate, ", ", avg_time, ", ", sensor.sensor.pos_noise, "\n")
write(f, res_str)
close(f)

using DataFrames
convergence_file="conv_qmdp_$p_birth$collision_cost.csv"
convergence_data = DataFrame([crashes, successes, time_outs, steps], [:crashes, :successes, :time_outs, :avg_steps])
writetable(convergence_file, convergence_data)
