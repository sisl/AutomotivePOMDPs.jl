using Parameters, AutomotiveDrivingModels, POMDPs, DeepRL
using ArgParse

s = ArgParseSettings()
@add_arg_table s begin
    "--port"
        help = "set port for zmq communication"
        arg_type = Int
        default=9393
end
parsed_args = parse_args(ARGS, s)

include("occluded_crosswalk_env.jl")
include("helpers.jl")
include("pomdp_types.jl")
include("constant_pedestrian.jl")
include("generative_model.jl")

rng = MersenneTwister(1)
pomdp = OCPOMDP()

ip = "127.0.0.1"
port = parsed_args["port"]

DeepRL.run_env_server(ip, port, pomdp)
