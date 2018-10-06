# Run some basic tests to check that all the functions are correct
using Base.Test, ProgressMeter
using AutomotiveDrivingModels, POMDPs, GridInterpolations, StatsBase, POMDPToolbox

# include all the relevant files in the right order
include("occluded_intersection_env.jl");
include("pomdp_types.jl")
include("spaces.jl")
include("transition.jl")
include("observation.jl")

#### HELPER FOR TESTING #####

# check that state indexing is consistent
function check_indexing(pomdp::SingleOIPOMDP, state_space::Vector{SingleOIState})
    for (i,s) in enumerate(state_space)
        @test stateindex(pomdp, s) == i
    end
end

# custom type for ADM
mutable struct PomdpAction
    a::SingleOIAction
    pomdp::SingleOIPOMDP
end
# custom propagate for testing
function AutomotiveDrivingModels.propagate(veh::Vehicle, a::PomdpAction, roadway::Roadway, ΔT::Float64)
    states, probs = ego_transition(pomdp, veh.state, a.a, ΔT)
    return sample(states, Weights(probs))
end
# custom model for testing
mutable struct EgoTestModel <: DriverModel{PomdpAction}
    a::PomdpAction
end
Base.rand(model::EgoTestModel) = model.a

# test 0 acceleration
function test_no_motion(pomdp::SingleOIPOMDP)
    ego = Vehicle(ego_state(pomdp, pomdp.s_start, 0.), VehicleDef(), 1)
    scene = Scene()
    push!(scene, ego)
    models = Dict{Int, DriverModel}()
    models[1] = EgoTestModel(PomdpAction(SingleOIAction(0.0), pomdp))
    nticks = 100
    rec = SceneRecord(nticks+1, 0.1)
    simulate!(rec, scene, pomdp.env.roadway, models, nticks)
    @test rec[0][1].state == ego.state
    @test rec[0][1].state.posF.s == pomdp.s_start
end

# test max acceleration
function test_reaching_goal(pomdp::SingleOIPOMDP)
    ego = Vehicle(ego_state(pomdp, pomdp.s_start, 0.), VehicleDef(), 1)
    scene = Scene()
    push!(scene, ego)
    models = Dict{Int, DriverModel}()
    models[1] = EgoTestModel(PomdpAction(SingleOIAction(pomdp.max_acc), pomdp))
    nticks = 100
    rec = SceneRecord(nticks+1, 0.1)
    simulate!(rec, scene, pomdp.env.roadway, models, nticks)
    @test rec[0][1].state.posF.s == pomdp.s_goal
end

# Test that transition sums up to 1
function test_transition(pomdp::SingleOIPOMDP)
    states_to_test = rand(ordered_states(pomdp), 100)
    @showprogress 0.5 "Checking transitions for 100 random states..." for (i, s) in enumerate(states_to_test)
        for (j, a) in enumerate(iterator(actions(pomdp)))
            prob = 0.
            d = transition(pomdp, s, a)
            for (k, sp) in enumerate(ordered_states(pomdp))
                prob += pdf(d, sp)
            end
            if !(prob ≈ 1.0)
                println(" Transition sums up to : ", prob)
                println("   For state action pair : ", s, a)
                break
            end
        end
    end
end

# Test that transition sums up to 1
function test_full_transition(pomdp::SingleOIPOMDP)
    @showprogress 0.5 "Checking transitions..." for (i, s) in enumerate(ordered_states(pomdp))
        for (j, a) in enumerate(iterator(actions(pomdp)))
            prob = 0.
            d = transition(pomdp, s, a)
            for (k, sp) in enumerate(ordered_states(pomdp))
                prob += pdf(d, sp)
            end
            if !(prob ≈ 1.0)
                println(" Transition sums up to : ", prob)
                println("   For state action pair : ", s, a)
                break
            end
        end
    end
end

#Test that observations sum up to 1
function test_observation(pomdp::SingleOIPOMDP)
    @showprogress 0.5 "Checking observations..." for (i, sp) in enumerate(ordered_states(pomdp))
        for (j, a) in enumerate(iterator(actions(pomdp)))
            prob = 0.
            d = observation(pomdp, a, sp)
            for (k, o) in enumerate(ordered_observations(pomdp))
                prob += pdf(d, o)
            end
            if !(prob ≈ 1.0)
                println( " Observation sums up to : ", prob)
                println("   For state action pair : ", sp, a)
                break
            end
        end
    end
end





# fix rng for the test
rng = MersenneTwister(3);

# initialize the environment and the pomdp
println("Testing Constructors")
params = EnvParams()
env = SimpleInterEnv(params);
pomdp = SingleOIPOMDP(env = env)

#### TEST STATE SPACES
println("Testing state space")
state_space = states(pomdp)
@test !isempty(state_space)
@test n_states(pomdp) == length(state_space)
check_indexing(pomdp, state_space)
println("SingleOIPOMDP with default parameters has ", n_states(pomdp), " states")

#test helpers
@test off_the_grid(pomdp, get_off_the_grid(pomdp))
s_grid = get_ego_s_grid(pomdp)
v_grid = get_v_grid(pomdp)
@test n_car_states(pomdp) == length(car_states(pomdp))

#### TEST ACTION SPACE
println("Testing action space")
action_space = actions(pomdp)
@test !isempty(action_space)
@test n_actions(pomdp) == length(action_space)


### TEST DISTRIBUTION


test_no_motion(pomdp)
test_reaching_goal(pomdp)

#TODO Test Car Transition

test_observation(pomdp)
test_transition(pomdp)
test_full_transition(pomdp)

# Check solver requirements
