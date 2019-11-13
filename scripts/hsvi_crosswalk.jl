using Revise
using HSVI
using POMDPs
using POMDPPolicies
using BeliefUpdaters
using POMDPModelTools
using POMDPModels

pomdp = TigerPOMDP()

spomdp = SparseTabularPOMDP(pomdp)
b0 = POMDPPolicies.beliefvec(pomdp, length(states(pomdp)), initialstate_distribution(pomdp))
solver = HSVISolver(Vector(b0), 1e-3, 1e-3, SawtoothProjection())

policy = solve(solver, spomdp)

using ProfileView
using Profile 

Profile.clear()

@profile solve(solver, spomdp)

ProfileView.view()
