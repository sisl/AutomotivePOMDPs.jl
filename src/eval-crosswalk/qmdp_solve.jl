using POMDPs, StatsBase, POMDPToolbox, SARSOP, QMDP, AutomotiveDrivingModels, JLD, Parameters, Distributions

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

config = EvalConfig(time_out = 200,
                    n_ped=2,
                    n_episodes = 3)


params = EnvParams(ped_rate = 0.9)
env = CrosswalkEnv(params)

pomdp = OCPOMDP()
pomdp.collision_cost = float(ARGS[1])
println("Collision Cost: $(pomdp.collision_cost)")
solver = QMDPSolver(max_iterations=100, tolerance=0.001)
qmdp_policy = solve(solver, pomdp, verbose=true)

JLD.save("crosswalk_policy$(abs(pomdp.collision_cost)).jld", "qmdp_policy", qmdp_policy)
