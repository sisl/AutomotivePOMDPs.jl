# Some useful function for the multiple user occluded crosswalk environment



"""
    is_crash(scene::Scene)
return true if the ego car is in collision in the given scene, do not check for collisions between
other participants
"""
function is_crash(scene::Scene)
    ego = scene[findfirst(scene, EGO_ID)]
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
    return scene[findfirst(scene, EGO_ID)]
end

function get_lane(roadway::Roadway, vehicle::Vehicle)
    lane_tag = vehicle.state.posF.roadind.tag
    return roadway[lane_tag]
end
function get_lane(roadway::Roadway, vehicle::VehicleState)
    lane_tag = vehicle.posF.roadind.tag
    return roadway[lane_tag]
end

function get_end(lane::Lane)
    s_end = round(lane.curve[end].s)
    if s_end % 2 == 1
        s_end -= 1
    end
    return s_end
end
