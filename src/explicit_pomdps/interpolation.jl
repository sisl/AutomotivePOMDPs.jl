# a bunch of interpolation helpers
function interpolate_state(mdp::Union{CarMDP, PedMDP, PedCarMDP}, state::VehicleState)
    # interpolate longitudinal position and velocity
    if state.posG == mdp.off_grid
        return VehicleState[state], Float64[1.0]
    end
    lane = get_lane(mdp.env.roadway, state)
    grid = mdp._car_grid[lane.tag]
    real_state = SVector{2, Float64}(state.posF.s, state.v)
    idx, weights = interpolants(grid, real_state)
    n_pts = length(idx)
    states = Vector{VehicleState}(n_pts)
    probs = zeros(n_pts)
    for i=1:n_pts
        sg, vg = ind2x(grid, idx[i])
        states[i] = VehicleState(Frenet(lane, sg), mdp.env.roadway, vg)
        probs[i] = weights[i]
    end
    return states, probs
end


function conservative_interpolation(mdp::Union{CarMDP, PedMDP, PedCarMDP}, state::VehicleState)
    # interpolate position and velocity separately
    if state.posG == mdp.off_grid
        return VehicleState[state], Float64[1.0]
    end
    lane = get_lane(mdp.env.roadway, state)
    route = get_ego_route(mdp.env)
    l_space = get_discretized_lane(lane.tag, mdp.env.roadway, mdp.pos_res)
    pgrid = mdp._l_grid[lane.tag]
    pidx, pweights = interpolants(pgrid, [state.posF.s])
    junction = state.posF.s > l_space[end] && lane.tag != route[end] ? 1 : 0
    jweight = 0.
    next_lane = nothing
    if junction == 1
        if length(pweights) != 1
            println(pweights)
            println([ind2x(pgrid, p) for p in pidx])
        end
        @assert length(pweights) == 1
        @assert pweights[1] == 1.
        dist_to_end = abs(get_end(lane) - state.posF.s)
        dist_to_prev = abs(state.posF.s - ind2x(pgrid, pidx[1])[1])
        jweight = dist_to_end/(dist_to_end + dist_to_prev)
        pweights[1] =  dist_to_prev/(dist_to_end + dist_to_prev)
        for (i, l) in enumerate(route)
            if l == lane.tag
                next_lane = mdp.env.roadway[route[i+1]]
                break
            end
        end
    end
    # println([ind2x(pgrid, pi) for pi in pidx])
    vgrid = mdp._v_grid
    vidx, vweights = interpolants(vgrid, [state.v])
    n_pts = length(pidx)*length(vidx)+junction*length(vidx)
    states = Vector{VehicleState}(n_pts)
    probs = zeros(n_pts)
    i = 1
    for j = 1:length(pidx)
        for k = 1:length(vidx)
            vg = ind2x(vgrid, vidx[k])[1]
            sg = ind2x(pgrid, pidx[j])[1]
            states[i] = VehicleState(Frenet(lane, sg), mdp.env.roadway, vg)
            probs[i] = pweights[j]*vweights[k]
            i += 1
        end
    end
    if junction == 1
        for l = 1:length(vidx)
            vg = ind2x(vgrid, vidx[l])[1]
            sg = 0.
            states[i] = VehicleState(Frenet(next_lane, sg), mdp.env.roadway, vg)
            probs[i] = jweight*vweights[l]
            i += 1
        end
    end
    normalize!(probs, 1)
    return states, probs
end

function handle_junction(mdp, state)
    lane = get_lane(mdp.env.roadway, state)

end

# take into account heading as well
function interpolate_pedestrian(mdp::Union{PedMDP, PedCarMDP}, state::VehicleState)
    # interpolate longitudinal position and velocity
    if state.posG == mdp.off_grid
        return (state,), (1.0,)
    end
    lane = get_lane(mdp.env.ped_roadway, state)
    grid = mdp._ped_grid[lane.tag]
    real_state = SVector{3, Float64}(state.posF.s, state.v, state.posF.ϕ)
    idx, weights = interpolants(grid, real_state)
    n_pts = length(idx)
    states = Vector{VehicleState}(n_pts)
    probs = zeros(n_pts)
    for i=1:n_pts
        sg, vg, phig = ind2x(grid, idx[i])
        states[i] = VehicleState(Frenet(lane, sg, 0., phig), mdp.env.ped_roadway, vg)
        probs[i] = weights[i]
    end
    return states, probs
end