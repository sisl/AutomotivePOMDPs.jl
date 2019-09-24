using Revise
using AutomotivePOMDPs
using SARSOP
using POMDPs
using AutomotiveDrivingModels
using POMDPPolicies
using BeliefUpdaters


params = CrosswalkParams(obstacles_visible = true)
env = CrosswalkEnv(params)
pomdp = SingleOCPOMDP(env = env,
                      collision_cost=-1.5)

@show n_states(pomdp)
@show n_actions(pomdp)
@show n_observations(pomdp)

# solver = SARSOPSolver(precision = 1e-3,
#                       randomization=true,
#                       timeout = 1000.0,
#                       pomdp_filename = "crosswalk.pomdpx",
#                       policy_filename = "crosswalk.policy")

# policy = solve(solver, pomdp)

policy = load_policy(pomdp, "crosswalk.policy")

# b0 = initialstate_distribution(pomdp)

# a = action(policy, b0)
# actionvalues(policy, b0)

# using POMDPGifs, POMDPSimulators, Random

# rng = MersenneTwister(1);
# hr = HistoryRecorder(max_steps=40, rng=rng)
# s0 = initialstate(pomdp, rng)
# up = DiscreteUpdater(pomdp)
# b0 = initialstate_distribution(pomdp)
# hist = simulate(hr, pomdp, policy, up, b0, s0)
# makegif(pomdp, hist, filename=joinpath("./", "crosswalk.gif"), spec="s,a", fps=2);

includet("legacy/policy_plot.jl")



using POMDPTesting

trans_prob_consistency_check(pomdp)

obs_prob_consistency_check(pomdp)

ego = AutomotivePOMDPs.xv_to_state(pomdp, 5.0, 0.0)
ped = AutomotivePOMDPs.yv_to_state(pomdp, 1.0, 0.0)
crash = false

s = SingleOCState(crash, ego, ped)

od = observation(pomdp, SingleOCAction(-4.0), s)

ospace = observations(pomdp);
p = 0.0
for o in ospace
    global p
    p += pdf(od, o)
    if !(pdf(od, o) ≈ 0.0)
        println(o)
    end
end

for o in od.it
    println(o)
end

op = SingleOCObs(false, ego,AutomotivePOMDPs.yv_to_state(pomdp, 2.0, 0.0))

pdf(od, op)

findfirst(isequal(op), ospace)

for (i, o) in enumerate(ospace)
    if o.ego == ego && o.ped.posG.y ≈ 2.0
        println(o)
        println(i)
    end
end

pdf(od, ospace[22])

ospace[22].ped
op.ped

op.ped ≈ ospace[22].ped



sum(o.p)

o.it

t = transition(pomdp, s, SingleOCAction(2.0))

t.p

sum(t.p)

for sp in t.it
    println(sp)
end

sspace = states(pomdp);

p = 0.0
for sp in sspace
    global p
    p += pdf(o, sp)
    if pdf(o, sp) != 0.0
        println(sp)
    end
end

o.it

pedp = AutomotivePOMDPs.yv_to_state(pomdp, 2.0, 0.0)
sp = SingleOCState(false, ego, pedp)

findfirst(isequal(sp), sspace)

pdf(o, sp)

pdf(o, sp)

for (i, s) in enumerate(sspace)
    if s.ped.posG.y ≈ 2.0 && s.ego.posG.x ≈ 5.0
        println(s)
        println(i)
    end
end

a = findfirst(isequal(sp), sspace)

sp = t.it[end]

is_crash(pomdp, sp.ego, sp.ped)

collision_checker(sp.ego, sp.ped, pomdp.ego_type, pomdp.ped_type)

using AutoViz
AutoViz.set_render_mode(:fancy)

ego = Vehicle(sp.ego, pomdp.ego_type, EGO_ID)
ped = Vehicle(sp.ped, pomdp.ped_type, PED_ID)

render(Scene([ego, ped]), pomdp.env.roadway, cam=CarFollowCamera(1, 20.0))



sum(t.p) ≈ 1.0