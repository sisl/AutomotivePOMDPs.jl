
function get_colors(scene::Scene, ego_id::Int64 = 1)
    colors = Dict{Int64, Colorant}()
    for veh in scene
        colors[veh.id] = veh.id == ego_id ?  COLOR_CAR_EGO : COLOR_CAR_OTHER
    end
    return colors
end
