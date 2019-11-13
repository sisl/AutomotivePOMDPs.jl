using Revise
using AutomotivePOMDPs
using SARSOP
using QMDP
using POMDPs
using AutomotiveDrivingModels
using POMDPPolicies
using BeliefUpdaters
includet("scripts/old_policy_plot.jl")

params = CrosswalkParams(obstacles_visible = true)
env = CrosswalkEnv(params)
pomdp = SingleOCPOMDP(env = env,
                      no_ped_prob = 0.0,
                      p_birth = 0.3,
                      collision_cost=-1.5)

@show length(states(pomdp))
@show length(actions(pomdp))
@show length(observations(pomdp))

sarsop_solver = SARSOPSolver(precision = 1e-3,
                      randomization=true,
                      timeout = 8*3600.0,
                      pomdp_filename = "crosswalk.pomdpx",
                      policy_filename = "crosswalk_long.policy",
                      policy_interval = 3*3600.0)

# qmdp_solver = QMDPSolver(belres = 1e-4, verbose=true)

# qmdp_policy = solve(qmdp_solver, pomdp);

# pomdp.collision_cost = -1.5

sarsop_policy = solve(sarsop_solver, pomdp);

sarsop_policy = load_policy(pomdp, "crosswalk_long.policy");



# b0 = initialstate_distribution(pomdp)

# a = action(policy, b0)
# actionvalues(policy, b0)

using POMDPGifs, POMDPSimulators, Random

rng = MersenneTwister(1);
hr = HistoryRecorder(max_steps=40, rng=rng);
s0 = initialstate(pomdp, rng);
up = DiscreteUpdater(pomdp);
b0 = initialstate_distribution(pomdp);
hist = simulate(hr, pomdp, sarsop_policy, up, b0, s0);
makegif(pomdp, hist, filename=joinpath("./", "crosswalk.gif"), spec="s,a", fps=2);

# includet("legacy/policy_plot.jl")


function uncertainty_plots(pomdp, policy)
    plots = []
    for sig in [1e-3, 0.5, 1.0, 2.0]
        p = policy_plot(pomdp, policy, sig=sig, n_bins=5, n_pts=150)
        push!(plots, p)
    end
    return plots
end

qp = QMDPPolicy(qmdp_policy);
sp = SARSOPPolicy(sarsop_policy);
qmdp_plots = uncertainty_plots(pomdp, qp);
sarsop_plots = uncertainty_plots(pomdp, sp);
g = GroupPlot(4, 4, groupStyle="horizontal sep = 1.75cm, vertical sep = 1.5cm");
push!.(Ref(g), qmdp_plots);
push!.(Ref(g), sarsop_plots);
g;
save("qmdp_sarsop_policies.tex", g)
g

save("policy_plots2.pdf", g)

qmdp_plots[end]

sarsop_plots[1]


using Profile
using ProfileView
Profile.clear()


p = policy_plot(pomdp, sp, sig=1.0, n_bins=5, n_pts=100, v_ego=5.0)

ProfileView.view()

policy = sarsop_policy

v_ego = 5.0
v_ped = 1.0
X, Y = get_XY_grid(pomdp);
X = X[1:21]
grid = RectangleGrid(X, Y);
vals = get_grid_data(policy, grid, pomdp, v_ego, v_ped);
acts = actions(pomdp)
action_map2 = [acts[ci[1]].acc for ci in  vec(argmax(vals, dims=1))]
action_map2 = reshape(action_map2, (21, 11))
acts = action_map2
acts = acts'
acts = acts[end:-1:1,1:end]


acts2 = compute_acts3(pomdp, policy)



cmap = ColorMaps.RGBArrayMap(colormap("RdBu"))
PGFPlots.Image(acts, (X[1], X[end]), (Y[1], Y[end]), colormap=cmap)

value_plots = [];
for i=1:length(actions(pomdp))
    xyval = reshape(vals[i, :], (21, 11))
    xyval = xyval'
    xyval = xyval[end:-1:1,1:end]
    p = PGFPlots.Image(xyval, (X[1], X[end]), (Y[1], Y[end]), colormap=cmap)
    push!(value_plots, p)
end;
g = GroupPlot(4, 1);
push!.(Ref(g), value_plots);
g

xyval = reshape(maximum(vals, dims=1), (21, 11))
xyval = xyval'
xyval = xyval[end:-1:1,1:end]
p = PGFPlots.Image(xyval, (X[1], X[end]), (Y[1], Y[end]), colormap=cmap)

length(X)
length(Y)


v_ego = 5.0
v_ped = 1



PGFPlots.save("sarsop_cw.pdf", p)

# using POMDPTesting

# trans_prob_consistency_check(pomdp)

# obs_prob_consistency_check(pomdp)

# ego = AutomotivePOMDPs.xv_to_state(pomdp, 5.0, 0.0)
# ped = AutomotivePOMDPs.yv_to_state(pomdp, 1.0, 0.0)
# crash = false

# s = SingleOCState(crash, ego, ped)

# od = observation(pomdp, SingleOCAction(-4.0), s)

# ospace = observations(pomdp);
# p = 0.0
# for o in ospace
#     global p
#     p += pdf(od, o)
#     if !(pdf(od, o) ≈ 0.0)
#         println(o)
#     end
# end

# for o in od.it
#     println(o)
# end

# op = SingleOCObs(false, ego,AutomotivePOMDPs.yv_to_state(pomdp, 2.0, 0.0))

# pdf(od, op)

# findfirst(isequal(op), ospace)

# for (i, o) in enumerate(ospace)
#     if o.ego == ego && o.ped.posG.y ≈ 2.0
#         println(o)
#         println(i)
#     end
# end

# pdf(od, ospace[22])

# ospace[22].ped
# op.ped

# op.ped ≈ ospace[22].ped



# sum(o.p)

# o.it

# t = transition(pomdp, s, SingleOCAction(2.0))

# t.p

# sum(t.p)

# for sp in t.it
#     println(sp)
# end

# sspace = states(pomdp);

# p = 0.0
# for sp in sspace
#     global p
#     p += pdf(o, sp)
#     if pdf(o, sp) != 0.0
#         println(sp)
#     end
# end

# o.it

# pedp = AutomotivePOMDPs.yv_to_state(pomdp, 2.0, 0.0)
# sp = SingleOCState(false, ego, pedp)

# findfirst(isequal(sp), sspace)

# pdf(o, sp)

# pdf(o, sp)

# for (i, s) in enumerate(sspace)
#     if s.ped.posG.y ≈ 2.0 && s.ego.posG.x ≈ 5.0
#         println(s)
#         println(i)
#     end
# end

# a = findfirst(isequal(sp), sspace)

# sp = t.it[end]

# is_crash(pomdp, sp.ego, sp.ped)

# collision_checker(sp.ego, sp.ped, pomdp.ego_type, pomdp.ped_type)

# using AutoViz
# AutoViz.set_render_mode(:fancy)

# ego = Vehicle(sp.ego, pomdp.ego_type, EGO_ID)
# ped = Vehicle(sp.ped, pomdp.ped_type, PED_ID)

# render(Scene([ego, ped]), pomdp.env.roadway, cam=CarFollowCamera(1, 20.0))



# sum(t.p) ≈ 1.0
