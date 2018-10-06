using AutomotivePOMDPs, POMDPs
using Base.Test
using ProgressMeter

function test_index(mdp::PedCarMDP, state_space::Vector{PedCarMDPState})
    @showprogress for (i, s) in enumerate(state_space) 
        si = stateindex(mdp, s)
        if si != i
            println("bug for $i, index is $si")
            break
        end
    end
end

mdp = PedCarMDP(pos_res=3.0)

n_states(mdp)

@times state_space = states(mdp)


test_index(mdp, state_space)
