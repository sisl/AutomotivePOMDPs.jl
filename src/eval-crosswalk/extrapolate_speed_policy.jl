#= Implementation of the baseline policy from D. Zhao paper:
Evaluation of Automated Vehicles Encountering Pedestrians ata Unsignalized Crossings
=#

type ExtrapolateSpeed
    env::CrosswalkEnv
    max_acc::Float64
    max_dec::Float64
    Δt::Float64
    # states
    initial_state::Vehicle
end


function has_crossed(ego::VehicleState, env::CrosswalkEnv)
    crosswalk_x = env.params.roadway_length/2 - env.params.crosswalk_width/2
    return crosswalk_x - 0.5 < ego.posG.x
end

function action(policy::ExtrapolateSpeed, b::Vector{Vehicle}, verbose::Bool = true)
    cl = policy.env.params.crosswalk_length
    ego = b[1].state
    if verbose; println("ego pos: $(ego.posG.x) ; ego vel: $(ego.v)") end
    if !has_crossed(ego, env)
        println("has not crossed")
        max_t_complete = 0.
        uncertain = true
        for ped in b[2:end]
            if verbose; println("ped pos: $(ped.state.posG.y)") end
            if ped.state.posG.y < env.params.lane_width
                uncertain = false
                t_complete = (env.params.lane_width - ped.state.posG.y)/ped.state.v # the 0 is at the center of the bottom lane
            else
                t_complete = 0
            end
            if t_complete > max_t_complete
                max_t_complete = t_complete
            end
        end
        if verbose; println("time to complete: $max_t_complete") end
        dist_to_cw = env.params.roadway_length/2 - ego.posG.x
        desired_speed = min(dist_to_cw/max_t_complete, env.params.speed_limit)

        acc = max(min((desired_speed - ego.v)/policy.Δt, policy.max_acc), policy.max_dec)
        if uncertain # no pedestrian detected, decelerate
            dist_to_cw = env.params.roadway_length/2 - ego.posG.x
            desired_speed = -policy.max_dec*policy.Δt
            acc = max(min((desired_speed - ego.v)/policy.Δt, policy.max_acc), policy.max_dec)
            if verbose; println("uncertain") end
        end
    else
        desired_speed = env.params.speed_limit
        acc = max(min((desired_speed - ego.v)/policy.Δt, policy.max_acc), policy.max_dec)
    end
    if verbose
        println("desired speed: $desired_speed")
        println("Acceleration: $acc")
    end
    return acc
end

type ExtrapolateSpeedUpdater
    policy::ExtrapolateSpeed
end

"""
Trivial belief updater for the baseline policy
"""
function update(up::ExtrapolateSpeedUpdater, bold::Vector{Vehicle}, a::Float64, o::Vector{Vehicle})
    return o
end



##################################################################################################
#TODO A BUNCH OF THINGS TO CLEAN UP

function is_at_crosswalk(ego::VehicleState, env::CrosswalkEnv)
    crosswalk_x = env.params.roadway_length/2 - env.params.crosswalk_width/2
    if crosswalk_x - 0.5 < ego.posG.x < crosswalk_x + 0.5 && ego.v == 0.
        return true
    end
    return false
end

function move_to_crosswalk(ego::VehicleState, Δt::Float64, env::CrosswalkEnv)
    crosswalk_x = env.params.roadway_length/2 - env.params.crosswalk_width/2
    Δx = crosswalk_x - ego.posG.x
    println("Δx = $Δx")
    Δv = -ego.v
    a = -Δv^2/(0.5*Δx)
    if isnan(a)
        a = 0.
    end
    return a
end

function constant_deceleration(policy::waitAndGo)
    crosswalk_x = policy.env.params.roadway_length/2 - policy.env.params.crosswalk_width/2
    Δx = crosswalk_x - policy.initial_state.posG.x
    Δv = -policy.initial_state.v
    a = -Δv^2/(0.5*Δx)
    return a
end

function is_occluded(env::CrosswalkEnv, ego::VehicleState)
    m = length(env.obstacles)
    x_ped = env.params.roadway_length/2
    cl = env.params.crosswalk_length
    y_min = -cl/4
    angle = atan2(y_min - ego.posG.y, x_ped - ego.posG.x)
    ray = VecSE2(ego.posG.x, ego.posG.y, angle)
    for i= 1:m
        if is_colliding(ray, env.obstacles[i])
            return true
        end
    end
    return false
end


    #
    #
    # if policy.Go
    #     if verbose ; println("Go phase"); end
    #     return 1.5*policy.max_acc
    # end
    #

    #
    # # first phase: reach the crosswalk and stop there
    # x_brake = ego.v^2 / (2*abs(policy.max_dec)) # minimum braking distance
    # delta_x_max = v_max*policy.Δt
    # delta_x_max = 0.
    # # println("Minimum braking distance : $x_brake ; for speed : $(s.ego.v)")
    # if ego.posG.x < crosswalk_x - x_brake - delta_x_max
    #     if verbose ;println("Reaching phase"); end
    #     return policy.max_acc
    # elseif ego.posG.x > crosswalk_x - x_brake - delta_x_max && ego.v > 0.1 # threshold on v
    #     if verbose ;println("Braking phase"); end
    #     return max(policy.max_dec, -ego.v/policy.Δt)
    # end
    #
    # # second phase: watch for the pedestrian
    # off_the_grid = true
    # if length(b) > 1
    #     off_the_grid = false
    #     ped = b[2].state
    # end
    # # if ped.posG.y > policy.env.params.lane_width/2
    # #     policy.Go = true
    # #     return policy.max_acc
    # # if verbose; println("off the grid : $off_the_grid, $(ped.posG.y), $(policy.env.params.lane_width/2)"); end
    # if policy.isFree && policy.N == 0
    #     if verbose ;println("Check 2 pass"); end
    #     policy.Go = true
    #     return policy.max_acc
    # elseif !(off_the_grid || ped.posG.y > policy.env.params.lane_width/2)
    #     if verbose ; println("waiting phase, reset "); end
    #     policy.isFree = false
    #     policy.N = 4
    #     return 0.0
    # elseif policy.isFree && policy.N > 0
    #     if verbose ; println("Keep checking"); end
    #     policy.N -= 1
    #     return 0.0
    # elseif !policy.isFree && (off_the_grid || ped.posG.y > policy.env.params.lane_width/2)
    #     if verbose ; println("Check 1 pass"); end
    #     policy.isFree = true
    #     return 0.0
    # end
    # return 0.0
