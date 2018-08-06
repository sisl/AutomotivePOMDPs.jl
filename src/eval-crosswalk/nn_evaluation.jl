using POMDPs, StatsBase, POMDPToolbox, SARSOP, QMDP, PyCall
using AutomotiveDrivingModels, Parameters, ArgParse, AutoViz, Reel

s = ArgParseSettings()
@add_arg_table s begin
    "--weights"
        help = "the path to the NN weights to load"
        arg_type = String
        default = ""
end

parsed_args  = parse_args(ARGS, s)
weight_path = parsed_args["weights"]

include("../admenv/crosswalk/occluded_crosswalk_env.jl") # env config
include("eval-env/constant_pedestrian.jl") # pedestrian model
include("eval-env/pedestrian_flow.jl")
include("eval-env/simple_sensor.jl")
include("eval-env/ego_control.jl")
include("simulation.jl")
include("eval_config.jl") # eval config
include("baseline_policy.jl")
include("render_helpers.jl")

config = EvalConfig(n_episodes=100, time_out = 200, n_ped=1) # use default
params = EnvParams(ped_rate = 0.9)
env = CrosswalkEnv(params)
cam = FitToContentCamera(0.0)

include("../admenv/crosswalk/pomdp_types.jl")
include("../admenv/crosswalk/spaces.jl")
include("../admenv/crosswalk/distributions.jl")
include("../admenv/crosswalk/belief.jl")
include("../adm-workspace/adm_helpers.jl")
include("../admenv/crosswalk/decomposition.jl")
include("eval-env/qmdp_eval_model.jl")
include("eval-env/nn_eval_model.jl")
include("eval-env/continuous_observations.jl")

@pyimport tensorflow as tf
nn_wrapper = pyimport("a3c.nn_wrapper")

sess = tf.Session();
rng = MersenneTwister(3);
pomdp = OCPOMDP();
pomdp.pos_res = 0.5
pomdp.vel_res = 0.5
pomdp.p_birth = 0.2
up = OCUpdater(pomdp);
nn = nn_wrapper[:NNPolicy](sess, weight_path, n_actions(pomdp))
policy = NNPolicy(pomdp, nn);
sensor = POMDPSensor(config.n_ped, pomdp, SimpleSensor(0.1, 0.1))
d0 = initial_state_distribution(pomdp)
crash = 0
success = 0
time_out = 0
avg_step = 0
crash_flag = true
for i=1:config.n_episodes
    s0 = rand(config.rng, d0)
    ego_initial_state = s0.ego
    ego = Vehicle(ego_initial_state, VehicleDef(), 1)
    #TODO function initialize belief
    b0 = Dict{Int64, NNBelief}()
    for j=1:config.n_ped
        b0[j+1] = Float64[]
    end
    models = Dict{Int, DriverModel}()
    up = NNUpdater(pomdp);
    reset_policy!(policy)
    models[1] = CrosswalkDriver(5, 0, env, 0., b0, Dict{Int64, OCState}(), ego, sensor, policy, up);
    # todo function init_scene
    scene = Scene()
    push!(scene, ego)
    nticks = 200
    rec = SceneRecord(nticks+1, config.sim_dt)

    # execute the simulation
    simulate!(rec, scene, env, models, nticks, config.rng, config.n_ped, config.callbacks)
    avg_step += nframes(rec)

    # analyse output
    if is_crash(rec[0]) && crash_flag
        crash += 1
        duration, fps, render_rec = animate_record(rec)
        speed_factor = 1
        film = roll(render_rec, fps = fps*speed_factor, duration = duration/speed_factor)
        write("crash.mp4", film)
        crash_flag = false
        # println("Episode $i CRASH")
        # break
    elseif is_terminal(rec[0], env)
        success += 1
        # println("Episode $i SUCCESS")
    elseif nframes(rec) >= config.time_out
        time_out += 1
        # println("Episode $i TIME OUT")
    else
        # println("Episode $i nothing happened, frames: $(nframes(rec))")
    end
end
println("End of the simulation: Crashes $crash , Successes $success, Time out $time_out, Avg steps $(avg_step/config.n_episodes)")
