# Some useful function for the multiple user occluded crosswalk environment

"""
    is_crash(scene::Scene)
return true if the ego car is in collision in the given scene, do not check for collisions between
other participants
"""
function is_crash(scene::Scene)
    ego = scene[findfirst(scene, 1)]
    @assert ego.id == 1
    if ego.state.v ≈ 0
        return false
    end
    for veh in scene
        if veh.id != 1
            if is_colliding(ego, veh)
                return true
            end
        end
    end
    return false
end

"""
return the ego vehicle from a given scene
"""
function get_ego(scene::Scene)
    return scene[findfirst(scene, 1)]
end
