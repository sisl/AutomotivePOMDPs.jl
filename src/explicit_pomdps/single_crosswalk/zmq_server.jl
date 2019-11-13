using POMDPs, StatsBase, POMDPToolbox, QMDP, RLInterface, AutomotiveDrivingModels, AutoViz, SARSOP, Images
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
include("pomdp_types.jl")
include("spaces.jl")
include("transition.jl")
include("observation.jl")
include("belief.jl")
include("decomposition.jl")
include("adm_helpers.jl")
include("render_helpers.jl")

function add_noise(o::Vector{Float64})
    return o + 1e-3*randn()
end

function POMDPs.initialobs(pomdp::SingleOCPOMDP, s::SingleOCState, rng::AbstractRNG)
    o = gen(DDNNode(:o), pomdp, s, SingleOCAction(0.), s, rng)
    return o
end



pomdp = SingleOCPOMDP()
pomdp.p_birth = 0.3
pomdp.pos_res = 0.5
pomdp.vel_res = 0.5
pomdp.pos_obs_noise = 0.5
pomdp.vel_obs_noise = 0.5

ip = "127.0.0.1"
port = parsed_args["port"]

RLInterface.run_env_server(ip, port, pomdp)
