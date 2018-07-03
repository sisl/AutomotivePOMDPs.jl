using AutomotiveDrivingModels, AutomotivePOMDPs, POMDPs, ProgressMeter

params = UrbanParams(nlanes_main=1,
                     crosswalk_pos =  [VecSE2(6, 0., pi/2), VecSE2(-6, 0., pi/2), VecSE2(0., -5., 0.)],
                     crosswalk_length =  [14.0, 14., 14.0],
                     crosswalk_width = [4.0, 4.0, 3.1],
                     stop_line = 22.0)
env = UrbanEnv(params=params);

mdp = PedCarMDP(env = env, pos_res=3., vel_res=2., ped_birth=0.7, ped_type=VehicleDef(AgentClass.PEDESTRIAN, 1.0, 3.0))


egos = get_ego_states(mdp.env, mdp.pos_res, mdp.vel_res)

for (i, e) in enumerate(egos)
    @assert ind2ego(mdp.env, i, mdp.pos_res, mdp.vel_res) == e
end

routes = AutomotivePOMDPs.get_car_routes(mdp.env)

for route in routes 
    cars = get_car_states(mdp.env, route, mdp.pos_res, mdp.vel_res)
    for (i, c) in enumerate(cars)
        @assert ind2car(mdp.env, i, route, mdp.pos_res, mdp.vel_res) == c
    end
end

peds = get_ped_states(mdp.env, mdp.pos_res, mdp.vel_ped_res)

for (i, p) in enumerate(peds)
    if ind2ped(mdp.env, i, mdp.pos_res, mdp.vel_ped_res) != p
        println(i)
        break
    end
end

state_space = states(mdp)

@showprogress for (i, s) in enumerate(state_space)
    @assert ind2state(mdp, i) == s
end
