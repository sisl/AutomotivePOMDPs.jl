### Observation model  ############################################################################

function POMDPs.observation(pomdp::SingleOCFPOMDP, a::SingleOCFAction, sp::SingleOCFObs)
    
    states = SingleOCFObs[]
    sizehint!(states, 100);
    probs = Float64[] 
    sizehint!(probs, 100);
    

    #=
    states_vector = Vector{SVector}(27)
    cnt = 1
    for s=-0.2:0.2:0.2
        for t=-0.2:0.2:0.2
            for theta=-0.34:0.34:0.34
                for v=-0.2:0.2:0.2
                    states_vector[cnt] = SVector(sp.ego_y, sp.ego_v, sp.ped_s+s, sp.ped_T+t, sp.ped_theta+theta, sp.ped_v+v)
                    cnt += 1
                end
            end
        end
    end
    =#

    states_vector = Vector{SVector}(9)
    cnt = 1
    for s=-0.2:0.2:0.2
        for t=-0.2:0.2:0.2
            states_vector[cnt] = SVector(sp.ego_y, sp.ego_v, sp.ped_s+s, sp.ped_T+t, sp.ped_theta, sp.ped_v)
            cnt += 1
        end
    end


#states_vector = Vector{SVector}(1)
#states_vector[1] = SVector(sp.ego_y, sp.ego_v, sp.ped_s, sp.ped_T, sp.ped_theta, sp.ped_v)

    for state_vector in states_vector
        ind, weight = interpolants(pomdp.state_space_grid, state_vector)
        for i=1:length(ind)
            if weight[i] >= 0.01
                state = pomdp.state_space[ind[i]]
                push!(states, state)
                push!(probs, weight[i])
                #=
                if !(state in states) # check for doublons
                    push!(states, state)
                    push!(probs, weight[i])
                else
                    state_ind = find(x->x==state, states)
                    probs[state_ind] += weight[i]
                end
                =#
            end
        end
    end
    
    # add roughening
    if length(probs) > 1
        normalize!(probs, 1)
        probs += maximum(probs)
        normalize!(probs,1)
    end
    
    return SparseCat(states,probs)

end