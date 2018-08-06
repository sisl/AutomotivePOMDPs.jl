using POMDPs, StatsBase, POMDPToolbox, QMDP, AutomotiveDrivingModels, JLD, Parameters, ArgParse, SARSOP

s = ArgParseSettings()
@add_arg_table s begin
    "--rng"
        help = "set rng seed"
        arg_type = Int
        default = 0
end
parsed_args = parse_args(ARGS, s)
seed = parsed_args["rng"]


include("../admenv/crosswalk/occluded_crosswalk_env.jl") # env config
include("eval-env/constant_pedestrian.jl") # pedestrian model
include("eval-env/pedestrian_flow.jl")
include("eval-env/simple_sensor.jl")
include("eval-env/ego_control.jl")
include("simulation.jl")
include("eval_config.jl") # eval config

rng = MersenneTwister(seed)
config = EvalConfig(time_out = 200, rng = rng) # use default

params = EnvParams(ped_rate = 0.9)
env = CrosswalkEnv(params)

sensor = SimpleSensor(0.1, 0.1)

include("../admenv/crosswalk/pomdp_types.jl")
include("../admenv/crosswalk/spaces.jl")
include("../admenv/crosswalk/distributions.jl")
include("../admenv/crosswalk/belief.jl")
include("../adm-workspace/adm_helpers.jl")
include("../admenv/crosswalk/decomposition.jl")
include("eval-env/qmdp_eval_model.jl")

pomdp = OCPOMDP();
pomdp.pos_res = 0.5
pomdp.vel_res = 0.5
pomdp.p_birth = 0.5
up = OCUpdater(pomdp);

qmdp_policy = load("qmdp_policy.jld")
policy = qmdp_policy["qmdp_policy"]

policy = QMDPEval(env = env, pomdp = pomdp, policy = policy, sim_dt = config.sim_dt)

include("eval-env/continuous_observations.jl")
include("eval-env/state_estimation.jl")

d0 = initial_state_distribution(pomdp)
s0 = rand(config.rng, d0)
ego_initial_state = s0.ego
ego = Vehicle(ego_initial_state, VehicleDef(), 1)
b0 = Dict{Int64, OCDistribution}()
b0[2] = initial_state_distribution(pomdp)
# b0[3] = initial_state_distribution(pomdp)
# b0[4] = initial_state_distribution(pomdp)

models = Dict{Int, DriverModel}()
ego = Vehicle(ego_initial_state, VehicleDef(), 1)
up = MixedUpdater(pomdp, ConstantPedestrian())
reset_policy!(policy)
models[1] = CrosswalkDriver(Int(policy.Δt/policy.sim_dt), 0, env, 0., b0, Dict{Int64, OCState}(), ego, POMDPSensor(), policy, up);

scene = Scene()
push!(scene, ego)
nticks = 200
rec = SceneRecord(nticks+1, config.sim_dt)
# execute the simulation
simulate!(rec, scene, env, models, nticks, config.rng, config.callbacks)

# analyse output
crash = 0
success = 0
time_out = 0
if is_crash(rec[0])
    crash += 1
    println("Episode  CRASH")
elseif is_terminal(rec[0], env)
    success += 1
    println("Episode  SUCCESS")
elseif nframes(rec) >= config.time_out
    time_out += 1
    println("Episode  TIME OUT")
else
    println("Episode  nothing happened, frames: $(nframes(rec))")
end
