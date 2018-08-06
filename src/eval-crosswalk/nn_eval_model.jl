mutable struct NNPolicy <: Policy
    pomdp::OCPOMDP
    nn::PyObject
    nstack::Int64
    input_shape::Tuple{Int64, Int64, Int64}
    action_map::Vector{OCAction}


    NNPolicy(pomdp::OCPOMDP, nn::PyObject) = new(pomdp, nn, 4, (84,84,1), ordered_actions(pomdp))
end

mutable struct NNBelief
    n::Int64 # number of observation to stack
    obs::Array{Float64}
end

mutable struct NNUpdater <: Updater
    problem::OCPOMDP
    n::Int64
    input_shape::Tuple{Int64, Int64, Int64}

    NNUpdater(problem::OCPOMDP) = new(problem, 4, (84,84,1))
end

function POMDPs.update(bu::NNUpdater, bold::NNBelief, a::Float64, o::OCObs)
    @assert bu.n == bold.n
    o_mat = convert_o(o, bu.problem)
    w, h = size(o_mat)
    new_obs = zeros(w, h, bu.n)
    # dequeue and append new obs
    for i=1:bu.n-1
        new_obs[:,:,i] = bold.obs[:,:,i+1]
    end
    new_obs[:,:,bu.n] = o_mat
    return NNBelief(bu.n, new_obs)
end

const OFF_KEY = -1

function POMDPs.update(bu::NNUpdater, bold::Dict{Int64,NNBelief}, a::Float64, o::Dict{Int64,OCObs})
    bnew = Dict{Int64, NNBelief}()
    for oid in keys(o)
        if haskey(bold, oid) && oid != OFF_KEY
            bnew[oid] = update(up, bold[oid], a, o[oid])
        elseif oid == OFF_KEY
            bnew[OFF_KEY] = update(up, bold[OFF_KEY], a, o[oid])
        else # cars appeared
            bnew[oid] = update(up, bold[OFF_KEY], a, o[oid])
        end
    end
    return bnew
end


function initialize_off_belief(policy::NNPolicy, ego::VehicleState)
    ped = get_off_the_grid(policy.pomdp)
    o0 = OCState(false, ego, ped)
    b0 = Dict{Int64, NNBelief}()
    o_mat = convert_o(o0, policy.pomdp)
    w, h = size(o_mat)
    o_stacked = reshape(repmat(o_mat, 1, policy.nstack), (w, h, policy.nstack))
    b0[OFF_KEY] = NNBelief(policy.nstack, o_stacked)
    return b0
end

# function normalize_o(s::OCState, pomdp::OCPOMDP)
#     o_vec = zeros(4)
#     o_vec[1], o_vec[2] = s.ego.posG.x/pomdp.x_goal, s.ego.v/pomdp.env.params.speed_limit
#     o_vec[3] = s.ped.posG.y/pomdp.y_goal
#     o_vec[4] = s.ped.v
#     if isinf(o_vec[4])
#         o_vec[4] = 0.
#     end
#     o_vec[4] /= pomdp.env.params.ped_max_speed
#     return o_vec
# end
#
# function normalize_o(x::Float64, v_ego::Float64, y::Float64, v_ped::Float64, pomdp::OCPOMDP)
#     o_vec = zeros(4)
#     o_vec[1], o_vec[2] = x/pomdp.x_goal, v_ego/pomdp.env.params.speed_limit
#     o_vec[3] = y/pomdp.y_goal
#     o_vec[4] = v_ped
#     if isinf(o_vec[4])
#         o_vec[4] = 0.
#     end
#     o_vec[4] /= pomdp.env.params.ped_max_speed
#     return o_vec
# end

function POMDPs.action(policy::NNPolicy, b::NNBelief)
    ai = policy.nn[:action](b.obs)
    return policy.action_map[ai]
end

function POMDPs.value(policy::NNPolicy, b::NNBelief)
    return policy.nn[:value](b.obs)
end

function action(policy::Policy, b::Dict{Int64, NNBelief})
    return POMDPs.action(policy, b).acc
end


const NNDriverModel = CrosswalkDriver{Float64, Dict{Int64,NNBelief},Dict{Int64, OCObs},NNPolicy,NNUpdater,POMDPSensor}

function reset_model!(model::NNDriverModel,
                      ego::Vehicle)
        model.tick = 0
        model.a = 0.
        model.b = initialize_off_belief(model.policy, ego.state)
        model.o = Dict{Int64, OCObs}()
        model.ego = ego
        reset_policy!(model.policy)
end


function reset_policy!(policy::NNPolicy)
    policy.nn[:reset]()
end
