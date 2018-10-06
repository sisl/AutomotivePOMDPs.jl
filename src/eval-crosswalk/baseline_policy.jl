### Baseline policy for the occlusion scenario
# move forward
# stop
# check for N seconds
# if free go, else wait



@with_kw mutable struct WaitAndGo
    env::CrosswalkEnv = CrosswalkEnv()
    initialstate::Vehicle = initial_ego(env, MersenneTwister(1))
    max_acc::Float64 = 2.
    max_dec::Float64 = -4.0
    Δt::Float64 = 0.1
    N0::Int64 = 3# Number of steps to wait
    threshold::Float64 = 2.5
    dist_delta::Float64 = 0.
    verbose::Bool = false

    # states
    reaching::Bool = true
    wait::Bool = false
    go::Bool = false
    N::Int64 = 0.
end




function action(policy::WaitAndGo, b::Vector{Vehicle})
    verbose = policy.verbose
    crosswalk_x = policy.env.params.roadway_length/2 - policy.env.params.crosswalk_width - policy.dist_delta
    v_max = policy.env.params.speed_limit
    ego = b[1].state
    @assert b[1].id == 1

    init_dec = -policy.initialstate.state.v^2/(2*(crosswalk_x - policy.initialstate.state.posG.x))
    @assert init_dec > policy.max_dec # negative numbers

    acc = 0.

    if policy.reaching #TODO use a smarter controller
        # if ego.v*cos(ego.posG.θ) < 0.
        #     if verbose; println("adjusting") end
        #     acc = -ego.v*cos(ego.posG.θ)/policy.Δt
        if ego.v ≈ 0.
            acc = 0.
        else
            acc = init_dec
        end
        # acc = init_dec
        if verbose; println("reaching phase, acc = $acc") end
        if !policy.wait && (ego.v ≈ 0. && ego.posG.x > policy.initialstate.state.posG.x) # change state
            policy.reaching = false
            policy.wait = true
        end
        return acc
    end

    if !policy.go
        if policy.N == policy.N0
            if verbose; println("GO!") end
            policy.go = true
            policy.wait = false
        else
            # measure ttc
            ttc_min = Inf
            for ped in b[2:end]
                ttc = (-env.params.lane_width/2 - ped.state.posG.y)/ped.state.v
                if verbose; println("pedestrian $(ped.id): ttc $(ttc) ; v $(ped.state.v)") end
                if 0 < ttc < ttc_min
                    ttc_min = ttc
                end
            end
            if verbose; println("minimum ttc: $ttc_min ; $(length(b)-1) pedestrians detected") end
            # check
            if ttc_min > policy.threshold
                if verbose; println("keep checking $(policy.N)") end
                # policy.wait = false
                policy.N += 1
            else
                if verbose; println("reset") end
                policy.N = 0
            end
        end
    end

    if policy.go
        acc = policy.max_acc
        return acc
    end
    return 0.
end


mutable struct WaitAndGoUpdater
    policy::WaitAndGo
    problem::OCPOMDP
end

"""
Trivial belief updater for the baseline policy, just return the observation
"""
function POMDPs.update(up::WaitAndGoUpdater, bold::Vector{Vehicle}, a::Float64, o::Vector{Vehicle})
    return o
end

"""
    reset_policy!(policy::waitAndGo, ego::Vehicle)
Reset the state of the FSM, must be run before each simulation. Change in place the fields of the
policy
"""
function reset_policy!(policy::WaitAndGo, ego::Vehicle)
    policy.initialstate = ego
    policy.reaching = true
    policy.wait = false
    policy.go = false
    policy.N = 0
end

const WaitAndGoModel = CrosswalkDriver{Float64,Vector{Vehicle},Vector{Vehicle},WaitAndGo,WaitAndGoUpdater,SimpleSensor}

function reset_model!(model::WaitAndGoModel,
                             ego::Vehicle)
        model.tick = 0
        model.a = 0.
        model.b = Vehicle[]
        model.o = Vehicle[]
        model.ego = ego
        reset_policy!(model.policy, ego)
end
