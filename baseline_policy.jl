### Baseline policy for the occlusion scenario
# move forward
# stop
# check for N seconds
# if free go, else wait
# only account for 1 pedestrian
using Parameters

@with_kw type waitAndGo
    env::CrosswalkEnv = CrosswalkEnv()
    initial_state::Vehicle = initial_ego(env, MersenneTwister(1))
    max_acc::Float64 = 3.
    max_dec::Float64 = -4.0
    Δt::Float64 = 0.1
    N0::Int64 = 3# Number of steps to wait
    threshold::Float64 = 2.5
    dist_delta::Float64 = 0.5

    # states
    reaching::Bool = true
    wait::Bool = false
    go::Bool = false
    N::Int64 = 0.
end




function action(policy::waitAndGo, b::Vector{Vehicle}, verbose::Bool = false)
    crosswalk_x = policy.env.params.roadway_length/2 - policy.env.params.crosswalk_width - policy.dist_delta
    v_max = policy.env.params.speed_limit
    ego = b[1].state
    @assert b[1].id == 1

    init_dec = -policy.initial_state.state.v^2/(2*(crosswalk_x - policy.initial_state.state.posG.x))
    @assert init_dec > policy.max_dec # negative numbers

    acc = 0.

    if policy.reaching #TODO use a smarter controller
        if ego.v*cos(ego.posG.θ) < 0.
            if verbose; println("adjusting") end
            acc = -ego.v*cos(ego.posG.θ)/policy.Δt
        elseif ego.v ≈ 0.
            acc = 0.
        else
            acc = init_dec
        end
        # acc = init_dec
        if verbose; println("reaching phase, acc = $acc") end
        if !policy.wait && (ego.v ≈ 0. && ego.posG.x > policy.initial_state.state.posG.x) # change state
            policy.reaching = false
            policy.wait = true
        end
        return acc
    end

    if !policy.go
        # measure ttc
        ttc_min = Inf
        for ped in b
            ttc = (env.params.lane_width - ped.state.posG.y)/ped.state.v
            if 0 < ttc < ttc_min
                ttc_min = ttc
            end
        end
        if verbose; println("minimum ttc: $ttc_min") end
        # check
        if ttc_min > policy.threshold
            if verbose; println("keep checking $(policy.N)") end
            # policy.wait = false
            policy.N += 1
        else
            if verbose; println("reset") end
            policy.N = 0
        end
        if policy.N == policy.N0
            if verbose; println("GO!") end
            policy.go = true
        end
    end

    if policy.go
        acc = policy.max_acc
        return acc
    end
    return 0.
end


type waitAndGoUpdater
    policy::waitAndGo
end

"""
Trivial belief updater for the baseline policy, just return the observation
"""
function update(up::waitAndGoUpdater, bold::Vector{Vehicle}, a::Float64, o::Vector{Vehicle})
    return o
end
