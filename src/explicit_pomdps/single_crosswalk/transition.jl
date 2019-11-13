### Transition distribution
using GridInterpolations

function POMDPs.transition(pomdp::SingleOCPOMDP, s::SingleOCState, a::SingleOCAction, dt::Float64 = pomdp.ΔT)
    if s.crash || s.ego.posG.x >= pomdp.x_goal || isterminal(pomdp, s)
        return Deterministic(TERMINAL_STATE)
    end
    ## Find Ego states first
    ego_states, ego_probs = ego_transition(pomdp, s.ego, a, dt)

    ### Find pedestrian states
    ped_probs, ped_states = ped_transition(pomdp, s.ped, dt)

    n_next_states = length(ego_states)*length(ped_states)

    ### Total state
    next_states = Vector{SingleOCState}(undef, n_next_states)
    next_probs = zeros(n_next_states)
    ind = 1
    for (i,ego) in enumerate(ego_states)
        for (j,ped) in enumerate(ped_states)
            collision = collision_checker(ego, ped, pomdp.ego_type, pomdp.ped_type)
            next_states[ind] = SingleOCState(collision, ego, ped)
            next_probs[ind] = ped_probs[j]*ego_probs[i]
            ind += 1
        end
    end
    normalize!(next_probs, 1)
    return SparseCat(next_states, next_probs)
end

"""
    ego_transition(pomdp::SingleOCPOMDP, ego::VehicleState, a::SingleOCAction)
Returns the distribution over the possible future state for the ego car only
"""
function ego_transition(pomdp::SingleOCPOMDP, ego::VehicleState, a::SingleOCAction, dt::Float64 = pomdp.ΔT)
    x_ = ego.posG.x + ego.v*dt + 0.5*a.acc*dt^2
    if x_ <= ego.posG.x # no backup
        x_ = ego.posG.x
    end
    v_ = ego.v + a.acc*dt # no backup
    if v_ <= 0.
        v_ = 0.
    end

    grid = RectangleGrid(get_X_grid(pomdp), get_V_grid(pomdp)) #XXX must not allocate the grid at each function call, find better implementation
    index, weight = interpolants(grid, [x_, v_])
    n_pts = length(index)

    states = Array{VehicleState}(undef, n_pts)
    probs = Array{Float64}(undef, n_pts)
    for i=1:n_pts
        xg, vg = ind2x(grid, index[i])
        states[i] = xv_to_state(pomdp, xg, vg)
        probs[i] = weight[i]
    end
    return states, probs
end


"""
    ped_transition(pomdp::SingleOCPOMDP, ped::VehicleState)
Return the distribution over the possible future state for the pedestrian only
"""
function ped_transition(pomdp::SingleOCPOMDP, ped::VehicleState, dt::Float64 = pomdp.ΔT)
    states = VehicleState[]
    probs = Float64[]
    sizehint!(states, 8)
    env = pomdp.env

    if pomdp.no_ped
        return [1.0], [get_off_the_grid(pomdp)]
    end

    if off_the_grid(pomdp, ped) # appear with random speed or stay of the grid
        p_birth = pomdp.p_birth
        V_grid = LinRange(0., env.params.ped_max_speed, Int(floor(env.params.ped_max_speed/pomdp.vel_res)) + 1)
        Y_grid = get_Y_grid(pomdp)
        for v in V_grid
            for y in Y_grid[1:div(length(Y_grid),2)- 3]
                push!(states, yv_to_state(pomdp, y, v))
            end
        end
        probs = ones(length(states) + 1)
        probs[1:end - 1] .= p_birth/length(states)
        # add the off the grid state
        push!(states, get_off_the_grid(pomdp))
        probs[end] = 1.0 - p_birth
        normalize!(probs, 1)
        return probs, states
    end

    grid = RectangleGrid(get_Y_grid(pomdp), get_V_ped_grid(pomdp)) #XXX preallocate
    y_ = ped.posG.y + ped.v*dt
    if y_ > pomdp.y_goal
        return [1.0], [get_off_the_grid(pomdp)]
    end
    for v_noise in [-pomdp.v_noise, 0., pomdp.v_noise]
        v_ = 1.0 + v_noise
        ind, weight = interpolants(grid, [y_, v_])
        for i=length(ind)
            yg, vg = ind2x(grid, ind[i])
            state = yv_to_state(pomdp, yg, vg)
            if !(state in states) # check for doublons
                push!(states, state)
                push!(probs, weight[i])
            else
                state_ind = findfirst(isequal(state), states)
                probs[state_ind] += weight[i]
            end
        end
    end
    # add roughening
    normalize!(probs, 1)
    probs .+= maximum(probs)
    normalize!(probs, 1)
    return probs, states
end

###### HELPERS #####################################################################################

"""
Helper to generate ego state
"""
function xv_to_state(pomdp::SingleOCPOMDP, x::Float64, v::Float64)
    return VehicleState(VecSE2(x, 0., 0.), pomdp.env.roadway, v)
end

"""
Helper to generate pedestrian state
"""
function yv_to_state(pomdp::SingleOCPOMDP, y::Float64, v::Float64)
    x_ped = 0.5*(pomdp.env.params.roadway_length - pomdp.env.params.crosswalk_width + 1)
    return VehicleState(VecSE2(x_ped, y, pi/2),
                        pomdp.env.roadway, v)
end
