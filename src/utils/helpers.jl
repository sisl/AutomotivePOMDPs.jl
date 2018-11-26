# Some useful function for the multiple user occluded crosswalk environment


#### POMDP related stuff #################################################################################
# Dummy type since the action is given by the policy
mutable struct EgoDriver{A} <: DriverModel{A}
    a::A
end

Base.rand(model::EgoDriver) = model.a

mutable struct GenerativeDist
    problem::POMDP
end

function Base.rand(rng::AbstractRNG, d::GenerativeDist)
    return initialstate(d.problem, rng)
end


"""
    is_crash(scene::Scene)
return true if the ego car is in collision in the given scene, do not check for collisions between
other participants
"""
function is_crash(scene::Scene)
    ego = scene[findfirst(EGO_ID, scene)]
    for veh in scene
        if veh.id != 1
            if collision_checker(ego, veh) #&& !(ego.state.v == 0. && veh.def.class == AgentClass.PEDESTRIAN)
                return true
            end
        end
    end
    return false
end

"""
Remove vehicles that have reached an end scene or that have collided
"""
function clean_scene!(env::OccludedEnv, scene::Scene)
    exit_lanes = get_exit_lanes(env.roadway)
    for veh in scene
        if veh.id == 1
            continue
        end
        s = veh.state.posF.s
        lane = get_lane(env.roadway, veh)
        s_end = get_end(lane)
        if s >= s_end && lane ∈ exit_lanes
            deleteat!(scene, findfirst(veh.id, scene))
        elseif veh.def.class == AgentClass.PEDESTRIAN
            if isapprox(s, 0., atol=0.01) && veh.state.posF.ϕ ≈ float(π)
                deleteat!(scene, findfirst(veh.id, scene))
            elseif s >= s_end && veh.state.posF.ϕ ≈ 0.
                deleteat!(scene, findfirst(veh.id, scene))
            end
        end
    end
    for veh in scene
        for veh_ in scene
            if collision_checker(veh, veh_) &&
               veh.id != 1 && veh_.id != 1 &&
               veh.id != veh_.id &&
               veh_.def.class != AgentClass.PEDESTRIAN && veh.def.class != AgentClass.PEDESTRIAN &&
               !(min(veh_.state.v, veh.state.v) ≈ 0.)
                deleteat!(scene, findfirst(veh_.id, scene))
            end
        end
    end
end


"""
Heuristic to check if a vehicle can be added to a given scene
"""
function can_push(env, scene::Scene, car::Vehicle, delta_headway::Float64 = 2.0)
    car_lane = get_lane(env.roadway, car)
    min_headway = Inf
    headway = min_headway
    for veh in scene
        if is_colliding(car, veh) || car.id == veh.id
            return false
        end
        veh_lane = get_lane(env.roadway, veh)
        if veh_lane == car_lane
            headway = veh.state.posF.s - car.state.posF.s
        end
        if headway <= min_headway
            min_headway = headway
        end
    end
    if min_headway < delta_headway
        return false
    end
    return true
end


"""
return the ego vehicle from a given scene
"""
function get_ego(scene::Scene)
    return scene[findfirst(EGO_ID, scene)]
end


### ROADWAY STUFF

"""
    get_lane(roadway::Roadway, vehicle::Vehicle)
    get_lane(roadway::Roadway, vehicle::VehicleState)
return the lane where `vehicle` is in.
"""
function get_lane(roadway::Roadway, vehicle::Vehicle)
    lane_tag = vehicle.state.posF.roadind.tag
    return roadway[lane_tag]
end
function get_lane(roadway::Roadway, vehicle::VehicleState)
    lane_tag = vehicle.posF.roadind.tag
    return roadway[lane_tag]
end

"""
    get_end(lane::Lane)
return the end longitudinal position of a lane 
"""
function get_end(lane::Lane)
    s_end = round(lane.curve[end].s)
    if s_end % 2 == 1 #XXX hack
        s_end -= 1
    end
    return s_end
end



"""
return the list of entrance lanes, i.e. the one that does not have any entrances
"""
function get_start_lanes(roadway::Roadway)
    lanes = Lane[]
    for i=1:length(roadway.segments)
        for lane in roadway.segments[i].lanes
            if isempty(lane.entrances) && !is_crosswalk(lane) && !(lane.tag ∈ [LaneTag(15,1), LaneTag(16, 1), LaneTag(6,1), LaneTag(13, 1), LaneTag(14,1)])
                push!(lanes, lane)
            end
        end
    end
    return lanes
end

"""
    is_crosswalk(lane::Lane) 
returns true if a lane is a crosswalk
"""
is_crosswalk(lane::Lane) = lane.width > 3.0  #XXX hack

"""
    get_lanes(roadway::Roadway)
return all the lanes in a roadway that are not crosswalk and not in the ego path 
"""
function get_lanes(roadway::Roadway)
    lanes = Lane[]
    for i=1:length(roadway.segments)
        for lane in roadway.segments[i].lanes
            if !is_crosswalk(lane) && !(lane.tag ∈ [LaneTag(15,1), LaneTag(16, 1), LaneTag(6,1), LaneTag(13, 1), LaneTag(14,1)]) #XXX hack
                push!(lanes, lane)
            end
        end
    end
    return lanes
end

"""
    get_all_lanes(roadway::Roadway)
returns a list of all the lanes present in `roadway`
"""
function get_all_lanes(roadway::Roadway)
    lanes = Lane[]
    for i=1:length(roadway.segments)
        for lane in roadway.segments[i].lanes
            push!(lanes, lane)
        end
    end
    return lanes
end

"""
return the list of exit lanes, i.e. the one that does not have any exits
"""
function get_exit_lanes(roadway::Roadway)
    lanes = Lane[]
    for i=1:length(roadway.segments)
        for lane in roadway.segments[i].lanes
            if isempty(lane.exits)
                push!(lanes, lane)
            end
        end
    end
    return lanes
end

"""
Given a list of lanes returns the list of exits
"""
function get_exit_lanes(lanes::Vector{Lane}, roadway::Roadway)
    exits = Lane[]
    for lane in lanes
        for exit in lane.exits
            tag = exit.target.tag
            new_exit = roadway[tag]
            if !(new_exit ∈ exits)
                push!(exits, new_exit)
            else
                continue
            end
        end
    end
    return exits
end

function lane_to_segment(lane::Lane, roadway::Roadway)
    # only works for straight lanes
    lane_a = lane.curve[1].pos #get_posG(Frenet(lane, 0.), roadway)
    lane_b = lane.curve[end].pos #get_posG(Frenet(lane, get_end(lane)), roadway)
    return LineSegment(lane_a, lane_b)
end

function  get_conflict_lanes(crosswalk::Lane, roadway::Roadway)
    # find lane intersecting with crosswalk
    cw_seg = lane_to_segment(crosswalk, roadway)
    conflict_lanes = Lane[]
    lanes = get_all_lanes(roadway)
    for lane in lanes
        if  is_crosswalk(lane)
            continue
        end
        lane_seg = lane_to_segment(lane, roadway)
        if intersects(lane_seg, cw_seg) && !(lane ∈ conflict_lanes)
            push!(conflict_lanes, lane)
        end
    end
    return conflict_lanes
end


"""
Generate a random route starting from start_lane to a random end node
"""
function random_route(rng::AbstractRNG, roadway::Roadway, start_lane::Lane)
    lanes = Lane[start_lane]
    cur_lane = start_lane
    while !isempty(cur_lane.exits)
        rand_exit = rand(rng, cur_lane.exits)
        next_lane_tag = rand_exit.target.tag
        next_lane = roadway[next_lane_tag]
        push!(lanes, next_lane)
        cur_lane = next_lane
    end
    return lanes
end

# return +1 if going toward, -1 if going away
function direction_from_center(ped::Vehicle, crosswalk::Lane)
    direction_from_center(ped.state, crosswalk)
end
function direction_from_center(ped::VehicleState, crosswalk::Lane)
    s_ped = ped.posF.s
    Δs = get_end(crosswalk)/2 - s_ped
    return sign(Δs*cos(ped.posF.ϕ))
end


struct LonAccelDirection
    a_lon::Float64
    direction::Int64
end

function Base.hash(a::LonAccelDirection, h::UInt)
    return hash(a.a_lon, hash(a.direction, h))
end
function Base.:(==)(a1::LonAccelDirection, a2::LonAccelDirection)
    return a1.a_lon == a2.a_lon && a1.direction == a2.direction
end

function AutomotiveDrivingModels.propagate(veh::Entity{VehicleState, D, Int}, action::LonAccelDirection,  roadway::Roadway, Δt::Float64) where {D<:Union{VehicleDef, BicycleModel}}
    previousInd = veh.state.posF.roadind
    a_lon = action.a_lon
    v = veh.state.v
    a_lat = 0.

    v = veh.state.v
       ϕ = veh.state.posF.ϕ
      ds = v*cos(ϕ)
       t = veh.state.posF.t
      dt = v*sin(ϕ)

      ΔT = Δt
      ΔT² = ΔT*ΔT
      Δs = ds*ΔT + 0.5*a_lon*ΔT²
      Δs = max(0, Δs) # no backup
      Δt = dt*ΔT + 0.5*a_lat*ΔT²

      ds₂ = ds + a_lon*ΔT
    ds₂ = max(0, ds₂) # no backup
      dt₂ = dt + a_lat*ΔT
      speed₂ = sqrt(dt₂*dt₂ + ds₂*ds₂)
      v₂ = sign(ds₂)*speed₂
      ϕ₂ = atan(dt₂, ds₂) + (v₂ < 0.0)*π # handle negative speeds
      roadind = AutoUrban.move_along_with_direction(veh.state.posF.roadind, roadway, Δs, direction = action.direction)
      footpoint = roadway[roadind]
      posF = Frenet(roadind, roadway, t=t+Δt, ϕ = ϕ₂)
      # posG = convert(VecE2, footpoint.pos) + polar(t + Δt, footpoint.pos.θ + π/2)
      # posG = VecSE2(posG.x, posG.y, footpoint.pos.θ + ϕ₂)

    state = VehicleState(posF, roadway, v₂)
    # state = VehicleState(posG, roadway[roadind.tag], roadway, v₂)
    return state
end

function AutomotiveDrivingModels.propagate(veh::VehicleState, action::LonAccelDirection,  roadway::Roadway, Δt::Float64)
    previousInd = veh.posF.roadind
    a_lon = action.a_lon
    v = veh.v
    a_lat = 0.

    v = veh.v
       ϕ = veh.posF.ϕ
      ds = v*cos(ϕ)
       t = veh.posF.t
      dt = v*sin(ϕ)

      ΔT = Δt
      ΔT² = ΔT*ΔT
      Δs = ds*ΔT + 0.5*a_lon*ΔT²
      Δs = max(0, Δs) # no backup
      Δt = dt*ΔT + 0.5*a_lat*ΔT²

      ds₂ = ds + a_lon*ΔT
    ds₂ = max(0, ds₂) # no backup
      dt₂ = dt + a_lat*ΔT
      speed₂ = sqrt(dt₂*dt₂ + ds₂*ds₂)
      v₂ = sign(ds₂)*speed₂
      ϕ₂ = atan(dt₂, ds₂) + (v₂ < 0.0)*π # handle negative speeds
      roadind = AutoUrban.move_along_with_direction(veh.posF.roadind, roadway, Δs, direction = action.direction)
      footpoint = roadway[roadind]
      posF = Frenet(roadind, roadway, t=t+Δt, ϕ = ϕ₂)
      posG = convert(VecE2, footpoint.pos) + polar(t + Δt, footpoint.pos.θ + π/2)
      posG = VecSE2(posG.x, posG.y, footpoint.pos.θ + ϕ₂)

    state = VehicleState(posG, posF, v₂)
    # state = VehicleState(posG, roadway[roadind.tag], roadway, v₂)
    return state
end

mutable struct LonAccel
    a_lon::Float64
end

function AutomotiveDrivingModels.propagate(veh::Entity{VehicleState, D, Int}, action::LonAccel,  roadway::Roadway, Δt::Float64) where {D<:Union{VehicleDef, BicycleModel}}
    a_lat = action.a_lat
   a_lon = action.a_lon

    v = veh.state.v
    ϕ = veh.state.posF.ϕ
   ds = v*cos(ϕ)
    t = veh.state.posF.t
   dt = v*sin(ϕ)

   ΔT² = ΔT*ΔT
   Δs = ds*ΔT + 0.5*a_lon*ΔT²
   Δt = dt*ΔT + 0.5*a_lat*ΔT²

   ds₂ = ds + a_lon*ΔT
   dt₂ = dt + a_lat*ΔT
   speed₂ = sqrt(dt₂*dt₂ + ds₂*ds₂)
   v₂ = sqrt(dt₂*dt₂ + ds₂*ds₂) # v is the magnitude of the velocity vector
   ϕ₂ = atan(dt₂, ds₂)

   roadind = move_along(veh.state.posF.roadind, roadway, Δs)
   footpoint = roadway[roadind]

   posG = convert(VecE2, footpoint.pos) + polar(t + Δt, footpoint.pos.θ + π/2)

   posG = VecSE2(posG.x, posG.y, footpoint.pos.θ + ϕ₂)

   return VehicleState(posG, roadway, v₂)
end
