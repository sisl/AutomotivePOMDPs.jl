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

