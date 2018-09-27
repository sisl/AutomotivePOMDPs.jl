### Observation model  ############################################################################

function POMDPs.observation(pomdp::SingleOCFPOMDP, a::SingleOCFAction, sp::SingleOCFState)
    
    states = SingleOCFObs[]
    sizehint!(states, 100);
    probs = Float64[] 
    sizehint!(probs, 100);
    
    
    
    push!(states, sp)
    push!(probs, 1)
   #=
    if !is_observable_fixed(sp, pomdp.env) || off_the_grid(pomdp, sp.ped)
        o = SingleOCObs(false, sp.ego, get_off_the_grid(pomdp))
        return SingleOCDistribution([1.0], [o])
    elseif is_crash(pomdp, sp)
        return SingleOCDistribution([1.0], [sp])
    end
    ego = sp.ego
    ped = sp.ped

    neighbors = Vector{VehicleState}(9)
    neighbors[1] = yv_to_state(pomdp, ped.posG.y - pomdp.pos_res, ped.v)
    neighbors[2] = yv_to_state(pomdp, ped.posG.y + pomdp.pos_res, ped.v)
    neighbors[3] = yv_to_state(pomdp, ped.posG.y - pomdp.pos_res, ped.v - pomdp.vel_res)
    neighbors[4] = yv_to_state(pomdp, ped.posG.y + pomdp.pos_res, ped.v - pomdp.vel_res)
    neighbors[5] = yv_to_state(pomdp, ped.posG.y - pomdp.pos_res, ped.v + pomdp.vel_res)
    neighbors[6] = yv_to_state(pomdp, ped.posG.y + pomdp.pos_res, ped.v + pomdp.vel_res)
    neighbors[7] = yv_to_state(pomdp, ped.posG.y, ped.v - pomdp.vel_res)
    neighbors[8] = yv_to_state(pomdp, ped.posG.y, ped.v + pomdp.vel_res)
    neighbors[9] = yv_to_state(pomdp, ped.posG.y, ped.v)


    states = SingleOCObs[]
    sizehint!(states, 9)
    for neighbor in neighbors
        if in_bounds_ped(pomdp, neighbor) && !is_crash(pomdp, ego, neighbor)
            push!(states, SingleOCObs(false, ego, neighbor))
        end
    end
    probs = zeros(length(states))
    for (i, o) in enumerate(states)
        probs[i] = obs_weight(o, sp, pomdp)
    end
    probs = normalize!(probs, 1)
    @assert length(probs) == length(states)
    return SingleOCDistribution(probs, states)
    =#
    
    
    # add roughening
    if length(probs) > 1
        normalize!(probs, 1)
        probs += maximum(probs)
        normalize!(probs,1)
    end
    
    return SparseCat(states,probs)

end