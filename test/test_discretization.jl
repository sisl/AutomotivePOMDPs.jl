mdp = PedMDP(env = env)

function test_ego_space(mdp::PedMDP)
    ego_space = get_ego_states(mdp.env, mdp.pos_res, mdp.vel_res);
    for (i, s) in enumerate(ego_space)
        if i != ego_state_index(env, s, mdp.pos_res, mdp.vel_res)
            return false
        end
    end
    return true
end

function test_car_space(mdp::PedMDP)
    car_space = get_car_states(mdp.env, mdp.pos_res, mdp.vel_res);
    for (i, s) in enumerate(car_space)
        if i != car_state_index(env, s, mdp.pos_res, mdp.vel_res)
            return false
        end
    end
    return true
end

function test_ped_space(mdp::PedMDP)
    ped_space = get_ped_states(mdp.env, mdp.pos_res, mdp.vel_res);
    for (i, s) in enumerate(ped_space)
        if i != ped_state_index(env, s, mdp.pos_res, mdp.vel_res)
            return false
        end
    end
    return true
end

@test test_ego_space(mdp)
@test test_car_space(mdp)
@test test_ped_space(mdp)
