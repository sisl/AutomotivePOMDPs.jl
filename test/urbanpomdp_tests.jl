using AutomotiveDrivingModels
using POMDPs
using POMDPSimulators
using POMDPPolicies
using BeliefUpdaters
using RLInterface
using Random 

@testset begin "Urban POMDP"
    rng = MersenneTwister(1)

    params = UrbanParams(nlanes_main=1,
                     crosswalk_pos =[VecSE2(6, 0., pi/2), VecSE2(-6, 0., pi/2), VecSE2(0., -5., 0.)],
                     crosswalk_length =  [14.0, 14., 14.0],
                     crosswalk_width = [4.0, 4.0, 3.1],
                     stop_line = 22.0)
    env = UrbanEnv(params=params)
    pomdp = UrbanPOMDP(env=env,
                   ego_goal = LaneTag(2, 1),
                   max_cars=5, 
                   max_peds=5, 
                   car_birth=0.5, 
                   ped_birth=0.2, 
                   max_obstacles=0, # no fixed obstacles
                   ego_start=20,
                   ΔT=0.1);

    up = KMarkovUpdater(4)

    policy = RandomPolicy(pomdp, rng=rng)

    hr = HistoryRecorder(rng=rng, max_steps = 100)
    s0 = initialstate(pomdp, rng)
    initial_observation = generate_o(pomdp, s0, rng)
    initial_obs_vec = fill(initial_observation, 4)
    hist = simulate(hr, pomdp, policy, up, initial_obs_vec, s0)

    @test n_steps(hist) > 1
end
