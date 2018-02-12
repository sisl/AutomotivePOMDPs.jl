### Implement transition and observation distributions

function POMDPs.transition(pomdp::SingleOIPOMDP, s::SingleOIState, a::SingleOIAction,
                           dt::Float64 = pomdp.ΔT,
                           pos_res::Float64 = pomdp.pos_res,
                           vel_res::Float64 = pomdp.vel_res)
    ## Find Ego states first
    ego_states, ego_probs = ego_transition(pomdp, s.ego, a, dt, pos_res, vel_res)

    ### Find pedestrian states
    car_states, car_probs = car_transition(pomdp, s.car, dt, pos_res, vel_res)

    n_next_states = length(ego_states)*length(car_states)

    ### Total state
    next_states = Vector{SingleOIState}(n_next_states)
    next_probs = zeros(n_next_states)
    ind = 1
    for (i,ego) in enumerate(ego_states)
        for (j,car) in enumerate(car_states)
            crash = is_crash(pomdp, ego, car)
            next_states[ind] = SingleOIState(crash, ego, car)
            next_probs[ind] = car_probs[j]*ego_probs[i]
            ind += 1
        end
    end
    normalize!(next_probs, 1)
    return SingleOIDistribution(next_probs, next_states)
end

"""
    ego_transition(pomdp::OCPOMDP, ego::VehicleState, a::OCAction)
Returns the distribution over the possible future state for the ego car only
"""
function ego_transition(pomdp::SingleOIPOMDP, ego::VehicleState, a::SingleOIAction, dt::Float64,
                        pos_res::Float64 = pomdp.pos_res,
                        vel_res::Float64 = pomdp.vel_res)
    s_ = ego.posF.s + ego.v*cos(ego.posF.ϕ)*dt + 0.5*a.acc*dt^2
    if s_ <= ego.posF.s # no backup
        s_ = ego.posF.s
    end
    v_ = ego.v + a.acc*dt # no backup
    if v_ <= 0.
        v_ = 0.
    end
    # interpolate in the grid
    grid = RectangleGrid(get_ego_s_grid(pomdp, pos_res), get_v_grid(pomdp, vel_res))
    index, weight = interpolants(grid, [s_, v_])
    n_pts = length(index)

    states = Array{VehicleState}(n_pts)
    probs = Array{Float64}(n_pts)
    for i=1:n_pts
        sg, vg = ind2x(grid, index[i])
        states[i] = ego_state(pomdp, sg, vg)
        probs[i] = weight[i]
    end
    return states, probs
end

"""
    car_transition(pomdp::SingleOIPOMDP, car::VehicleState, dt::Float64)
return the next possible state and their probability for any other car
"""
function car_transition(pomdp::SingleOIPOMDP, car::VehicleState, dt::Float64,
                        pos_res::Float64 = pomdp.pos_res,
                        vel_res::Float64 = pomdp.vel_res)
    env = pomdp.env

    v_grid = get_v_grid(pomdp, vel_res)

    states = VehicleState[]
    probs = Float64[]
    sizehint!(states, 8)
    lane = env.roadway[car.posF.roadind.tag]
    if off_the_grid(pomdp, car) || car.posF.s >= get_end(lane) # appear with random speed or stay of the grid
        p_birth = pomdp.p_birth
        for v in v_grid
            for lane in LANE_ID_LIST
                push!(states, car_state(pomdp, lane, 0., v))
            end
        end
        probs = ones(length(states) + 1)
        probs[1:end - 1] = p_birth/length(states)
        # add the off the grid state
        push!(states, get_off_the_grid(pomdp))
        probs[end] = 1.0 - p_birth
        normalize!(probs, 1)
        return states, probs
    end
    s_grid = get_lane_s(pomdp, lane, pos_res)
    grid = RectangleGrid(s_grid, v_grid)
    s_ = car.posF.s + car.v*cos(car.posF.ϕ)*dt
    if s_ > get_end(lane)
        return [get_off_the_grid(pomdp)], [1.0]
    end
    for a_noise in [-pomdp.a_noise, 0., pomdp.a_noise]
        s_ += 0.5*a_noise*dt^2/2
        v_ = car.v + a_noise*dt
        ind, weight = interpolants(grid, [s_, v_])
        for i=length(ind)
            sg, vg = ind2x(grid, ind[i])
            state = car_state(pomdp, lane, sg, vg)
            if !(state in states) # check for doublons
                push!(states, state)
                push!(probs, weight[i])
            else
                state_ind = find(x->x==state, states)
                probs[state_ind] += weight[i]
            end
        end
    end
     # add roughening
    normalize!(probs, 1)
    probs += maximum(probs)
    normalize!(probs)
    return states, probs
end
