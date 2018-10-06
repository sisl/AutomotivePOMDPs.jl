using Parameters, AutomotiveDrivingModels, POMDPs

include("occluded_crosswalk_env.jl")
include("helpers.jl")
include("pomdp_types.jl")
include("constant_pedestrian.jl")
include("generative_model.jl")

rng = MersenneTwister(1)
pomdp = OCPOMDP()


s0 = initialstate(pomdp, rng)
ego = s0[findfirst(EGO_ID,s0)]
println("Initial scene with $(length(s0)) cars")
println("Initial ego state: $ego")
