# a bunch of overlays for debugging
mutable struct BeliefOverlay <: SceneOverlay
    verbosity::Int
    color::Colorant
    font_size::Int
    belief::Any
    policy::DecomposedPolicy

    function BeliefOverlay(belief, policy::DecomposedPolicy,
        verbosity::Int=1;
        color::Colorant=colorant"white",
        font_size::Int=20
        )

        new(verbosity, color,font_size, belief, policy)
    end
end

function AutoViz.render!(rendermodel::RenderModel, overlay::BeliefOverlay, scene::Scene, env::OccludedEnv)
    AutoViz.render!(rendermodel, overlay.belief, overlay, scene, env)
    return rendermodel
end

function AutoViz.render!(rendermodel::RenderModel, belief::PreviousObsDecBelief, overlay::BeliefOverlay, Scene::Scene, env::OccludedEnv)
    for (prob_key, b) in belief
        n_obstacles = 0
        if prob_key == :obsped || prob_key == :obscar
            n_obstacles = 3
        end
        problem = overlay.policy.problem_map[prob_key]
        AutoViz.render!(rendermodel, overlay,n_obstacles, problem, b, env)
    end
end

function AutoViz.render!(rendermodel::RenderModel, belief::KMarkovDecBelief, overlay::BeliefOverlay, Scene::Scene, env::OccludedEnv)
    for (prob_key, b) in belief
        n_obstacles = 0
        if prob_key == :obsped || prob_key == :obscar
            n_obstacles = 3
        end
        problem = overlay.policy.problem_map[prob_key]
        for i=1:size(b, 2)
            AutoViz.render!(rendermodel, overlay,n_obstacles, problem, b[:,i][:], env)
        end
    end
end


function AutoViz.render!(rendermodel::RenderModel, overlay::BeliefOverlay, n_obstacles::Int64, problem::UrbanPOMDP, b::Vector{Float64}, env::OccludedEnv, transparency = 0.1)
    o = deepcopy(b)
    o = unrescale!(o, problem)
    ego, car_map, ped_map, obs_map = AutomotivePOMDPs.split_o(o, problem, 4, n_obstacles)
    for (key, state) in car_map
        x, y, θ, v = state
        color = color=RGBA(75.0/255, 66.0/255, 244.0/255, transparency)
        add_instruction!(rendermodel, render_vehicle, (x, y, θ, problem.car_type.length, problem.car_type.width, color, color, RGBA(1.,1.,1.,transparency)))
    end
    for (key, state) in ped_map
        x, y, θ, v = state
        color = RGBA(75.0/255, 66.0/255, 244.0/255, transparency)
        add_instruction!(rendermodel, render_vehicle, (x, y, θ, problem.ped_type.length, problem.ped_type.width, color, color, RGBA(1.,1.,1.,transparency)))
    end
end

function AutomotivePOMDPs.animate_scenes(scenes::Vector{Scene},
                        actions::Vector{Float64},
                        beliefs,
                        policy::DecomposedPolicy,
                        env;
                        overlays::Vector{SceneOverlay} = SceneOverlay[],
                        cam=FitToContentCamera(0.),
                        sim_dt=0.1)
    duration =length(scenes)*sim_dt
    fps = Int(1/sim_dt)
    function render_rec(t, dt)
        frame_index = Int(floor(t/dt)) + 1
        return AutoViz.render(scenes[frame_index],
                              env,
                              cat(1, overlays,
                                     TextOverlay(text = ["Acc: $(actions[frame_index])"]),
                                     BeliefOverlay(beliefs[frame_index], policy)),
                              cam=cam,
                              car_colors=get_colors(scenes[frame_index]))
    end
    return duration, fps, render_rec
end
