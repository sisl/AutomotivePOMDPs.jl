using POMDPs
using Random
using POMDPSimulators
using RLInterface
using BeliefUpdaters
using POMDPPolicies
using POMDPTesting

@testset begin "single crosswalk"
    params = CrosswalkParams(obstacles_visible = true)
    env = CrosswalkEnv(params)
    pomdp = SingleOCPOMDP(env = env,
                        collision_cost=-1.5)
    @test trans_prob_consistency_check(pomdp)

    @test obs_prob_consistency_check(pomdp)
end

@testset begin "crosswalk"
    rng = MersenneTwister(1)

    pomdp = OCPOMDP(ΔT = 0.5, p_birth = 0.3, max_peds = 1)

    env = KMarkovEnvironment(pomdp, k=4)

    up = KMarkovUpdater(4)

    policy = RandomPolicy(pomdp, rng=rng)

    hr = HistoryRecorder(rng=rng, max_steps = 100)
    s0 = initialstate(pomdp, rng)
    initial_observation = generate_o(pomdp, s0, rng)
    initial_obs_vec = fill(initial_observation, 4)
    hist = simulate(hr, pomdp, policy, up, initial_obs_vec, s0)

    @test n_steps(hist) > 1
end