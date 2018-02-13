
function animate(t, dt)
    frame_index = Int(floor(t/dt)) + 1
    return render(rec[frame_index-nframes(rec)], env.roadway, [CarFollowingStatsOverlay(1)], cam=cam)
end

function animate_scenes(scenes::Vector{Scene})
    duration =length(scenes)*config.sim_dt
    fps = Int(1/config.sim_dt)
    cam = FitToContentCamera(0.)
    function render_rec(t, dt)
        frame_index = Int(floor(t/dt)) + 1
        return render(scenes[frame_index], env, cam=cam)
    end
    return duration, fps, render_rec
end

function animate_scenes(scenes::Vector{Scene}, overlay::SceneOverlay)
    duration =length(scenes)*config.sim_dt
    fps = Int(1/config.sim_dt)
    cam = FitToContentCamera(0.)
    function render_rec(t, dt)
        frame_index = Int(floor(t/dt)) + 1
        return render(scenes[frame_index], env, [overlay], cam=cam)
    end
    return duration, fps, render_rec
end

function animate_states(pomdp::SingleOCPOMDP, states::Vector{SingleOCState}, overlay=nothing)
    duration = length(states)*pomdp.ΔT
    fps = Int(1/pomdp.ΔT)
    cam = FitToContentCamera(0.)
    function render_states(t, dt)
        state_index = Int(floor(t/dt)) + 1
        scene = state_to_scene(pomdp, states[state_index])
        return AutoViz.render(scene, pomdp.env, cam = cam)
    end
    return duration, fps, render_states
end

function animate_hist{B}(pomdp::SingleOCPOMDP, hist::POMDPHistory{SingleOCState,SingleOCAction,SingleOCState, B})
    duration = n_steps(hist)*pomdp.ΔT
    fps = Int(1/pomdp.ΔT)
    cam = FitToContentCamera(0.)
    function render_hist(t, dt)
        state_index = Int(floor(t/dt)) + 1
        scene = state_to_scene(pomdp, state_hist(hist)[state_index])
        return AutoViz.render(scene, pomdp.env, cam = cam)
    end
    return duration, fps, render_hist
end

# function animate_record(rec::SceneRecord, overlay::SceneOverlay)
#     duration =rec.nframes*config.sim_dt
#     fps = Int(1/config.sim_dt)
#     function render_rec(t, dt)
#         frame_index = Int(floor(t/dt)) + 1
#         return render(rec[frame_index-nframes(rec)], env, [overlay], cam=cam)
#     end
#     return duration, fps, render_rec
# end
#
# mutable struct DQNFeatures
#     env::CrosswalkEnv
# end
#
# function AutoViz.render!(rendermodel::RenderModel, env_::DQNFeatures)
#     env = env_.env
#     render!(rendermodel, env.roadway)
#
#     obs = env.obstacles[1]
#     for obs in env.obstacles
#         pts = Array{Float64}(2, obs.npts)
#         for (i, pt) in enumerate(obs.pts)
#             pts[1,i] = pt.x
#             pts[2,i] = pt.y
#         end
#
#         add_instruction!(rendermodel, render_fill_region, (pts, colorant"gray"))
#     end
# end
#
# mutable struct InflateOverlay <: SceneOverlay
# end
#
# function AutoViz.render!(rendermodel::RenderModel, overlay::InflateOverlay, scene::Scene, _env::DQNFeatures)
#     env = _env.env
#     ego = scene[findfirst(scene, 1)]
#     for veh in scene
#         # increase pedestrian size for rendering
#         if veh.def.class == AgentClass.PEDESTRIAN && is_observable(veh.state, ego.state, env)
#             add_instruction!(rendermodel, render_vehicle, (veh.state.posG.x, veh.state.posG.y, veh.state.posG.θ,
#                                               2,2,COLOR_CAR_OTHER))
#         end
#     end
#
#     front = ego.state.posG + polar(VehicleDef().length/2, ego.state.posG.θ)
#     # check if ego is in front of obstacle
#     if front.x < env.params.roadway_length/2 - env.params.crosswalk_width/2
#         obstacles = ConvexPolygon(4)
#         obstacles.pts = [VecSE2(22, -1.5), VecSE2(28, -1.5), VecSE2(28, -13), VecSE2(22, -13)]
#         obstacles.npts = 4; # a rectangle
#         for obs in obstacles
#             pts = Array{Float64}(2, obstacles.npts)
#             for (i, pt) in enumerate(obstacles.pts)
#                 pts[1,i] = pt.x
#                 pts[2,i] = pt.y
#             end
#
#             add_instruction!(rendermodel, render_fill_region, (pts, colorant"black"))
#         end
#     end
#
#     rendermodel
# end
