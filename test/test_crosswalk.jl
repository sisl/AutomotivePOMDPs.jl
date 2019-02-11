using Revise
using Printf
using Random
using POMDPs
using POMDPPolicies
using POMDPSimulators
using BeliefUpdaters
using AutomotiveDrivingModels
using AutomotivePOMDPs
using AutoViz
using RLInterface
using DeepQLearning

rng = MersenneTwister(1)

const K = 4

pomdp = OCPOMDP(ΔT = 0.5, p_birth = 0.3, max_peds = 1)

env = KMarkovEnvironment(pomdp, k=K)

up = KMarkovUpdater(K)

policy = RandomPolicy(pomdp, rng=rng)

hr = HistoryRecorder(rng=rng, max_steps = 100)
s0 = initialstate(pomdp, rng)
initial_observation = generate_o(pomdp, s0, rng)
initial_obs_vec = fill(initial_observation, K)
hist = simulate(hr, pomdp, policy, up, initial_obs_vec, s0)



# Visualize
using Reel
frames = Frames(MIME("image/png"), fps=4)
AutoViz.render(s0, pomdp.env, cam = StaticCamera(VecE2(25.0,0.0), 15.0))
for step in eachstep(hist, "s,a,r,sp")
    s, a, r, sp = step
    push!(frames, AutoViz.render(sp, pomdp.env, cam=StaticCamera(VecE2(25.0,0.0), 15.0)))
end
write("out.gif", frames)

pomdp.env.obstacles