
function get_colors(scene::Scene, ego_id::Int64 = 1)
    colors = Dict{Int64, Colorant}()
    for veh in scene
        colors[veh.id] = veh.id == ego_id ?  COLOR_CAR_EGO : COLOR_CAR_OTHER
    end
    return colors
end

function animate_record(env, rec::SceneRecord; sim_dt::Float64=0.1)
    duration =rec.nframes*sim_dt
    fps = Int(1/sim_dt)
    function render_rec(t, dt)
        frame_index = Int(floor(t/dt)) + 1
        return AutoViz.render(rec[frame_index-nframes(rec)], env, cam=cam)
    end
    return duration, fps, render_rec
end
