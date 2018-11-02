using POMDPs, POMDPToolbox, RLInterface, Parameters
using AutomotiveDrivingModels, AutoUrban, AutoViz, Reel
using ArgParse

include("../AutomotivePOMDPs.jl")
using AutomotivePOMDPs

s = ArgParseSettings()
@add_arg_table s begin
    "--port"
        help = "set port for zmq communication"
        arg_type = Int
        default=9394
end
parsed_args = parse_args(ARGS, s)

rng = MersenneTwister(1)
pomdp = OCPOMDP(max_peds=3, no_ped_prob=0.1)

ip = "127.0.0.1"
port = parsed_args["port"]

# env = KMarkovEnvironment(pomdp, k=4)
# RLInterface.run_env_server(ip, port, env)
RLInterface.run_env_server(ip, port, pomdp)
