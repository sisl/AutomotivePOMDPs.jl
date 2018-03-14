# helper functions to use adm along with POMDPs.jl

"""
    state_to_scene(pomdp::SingleOCPOMDP, s::SingleOCState)
Convert an SingleOCState object to a scene
"""
function state_to_scene(pomdp::SingleOCPOMDP, s::SingleOCState)
    scene = Scene()
    ego = Vehicle(s.ego, pomdp.ego_type, EGO_ID)
    ped = Vehicle(s.ped, pomdp.ped_type, 2)
    push!(scene, ego)
    push!(scene, ped)
    return scene
end

"""
    rand_scene(rng::AbstractRNG, b::Dict{Int64, SingleOCDistribution}, env::CrosswalkEnv)
Sample a scene from a belief representation
"""
function rand_scene(rng::AbstractRNG, b::Dict{Int64, SingleOCDistribution}, env::CrosswalkEnv)
    scene = Scene()
    for id in keys(b)
        s = rand(rng, b[id])
        if scene.n == 0
            ego = Vehicle(s.ego, pomdp.ego_type, EGO_ID)
            push!(scene, ego)
        end
        ped = Vehicle(s.ped, PEDESTRIAN_DEF, id)
        push!(scene, ped)
    end
    return scene
end

"""
    scene_to_states(scene::Scene, pomdp::SingleOCPOMDP)
Convert a scene to a state representation, for each pedestrian present in the scene, a state object is created
"""
function scene_to_states(scene::Scene, pomdp::SingleOCPOMDP)
    ego = scene[findfirst(scene, EGO_ID)]
    states = Dict{Int64, SingleOCState}()
    if scene.n == 1
        states[2] = SingleOCState(false, ego, get_off_the_grid(pomdp))
    end
    for i=1:scene.n - 1
        ped = scene[findfirst(scene, i+1)]
        states[ped.id] = SingleOCState(is_crash(pomdp, ego, ped.state), ego, ped.state)
    end
    return states
end

"""
    states_to_scene(states::Dict{Int64, SingleOCState}, pomdp::SingleOCPOMDP)
"""
function states_to_scene(states::Dict{Int64, SingleOCState}, pomdp::SingleOCPOMDP)
    scene = Scene()
    for id in keys(states)
        if scene.n == 0
            ego = Vehicle(states[id].ego, pomdp.ego_type, 1)
            push!(scene, ego)
        end
        ped = Vehicle(states[id].ped, PEDESTRIAN_DEF, id)
        push!(scene, ped)
    end
    return scene
end
