### CONFIGURATION OF THE OCCLUDED INTERSECTION ENVIRONMENT

const LANE_ID_LIST = ["right_from_left", "straight_from_right", "left_from_right", "straight_from_left"]

"""
Container type for the environment parameters
"""
mutable struct EnvParams
    # geometry
    n_lanes::Int64
    hor_roadway_length::Float64
    ver_roadway_length::Float64
    lane_width::Float64
    turn_radius::Float64
    stop_line::Float64
    # cars
    speed_limit::Float64
    car_rate::Float64
end

function EnvParams(;n_lanes::Int64 = 2,
                    hor_roadway_length::Float64 = 20.,
                    ver_roadway_length::Float64 = 10.,
                    lane_width::Float64 = 3.,
                    turn_radius::Float64 = 5.,
                    stop_line::Float64 = 8.,
                    speed_limit::Float64 = 8.,
                    car_rate::Float64 = 0.5)
    return EnvParams(n_lanes, hor_roadway_length, ver_roadway_length, lane_width, turn_radius,
                     stop_line, speed_limit, car_rate)
end

"""
Define the Intersection environment
"""
mutable struct IntersectionEnv <: OccludedEnv
    roadway::Roadway
    obstacles::Vector{ConvexPolygon}
    params::EnvParams
    lane_map::Dict{String, Lane}
    end_pos::Float64
end

"""
    build_t_shape(params::EnvParams)
generate a T-shape intersection using AutomotiveDrivingModels
"""
function build_t_shape(params::EnvParams)
    roadway = Roadway()
    BEZIER_SAMPLES = 51
    LANE_ID_MAP = Dict{String, Lane}()
    SPEED_LIMIT = SpeedLimit(0., params.speed_limit)
    # Define coordinates of the entry and exit points to the intersection
    r = params.turn_radius # turn radius
    A = VecSE2(0,params.lane_width,-π)
    B = VecSE2(0,0,0)
    C = VecSE2(r,-r,-π/2)
    D = VecSE2(r+params.lane_width,-r,π/2)
    E = VecSE2(2r+params.lane_width, 0, 0)
    F = VecSE2(2r+params.lane_width,params.lane_width,-π)

    # Append right turn coming from the left
    curve = gen_straight_curve(convert(VecE2, B+VecE2(-params.hor_roadway_length/2,0)),
                               convert(VecE2, B), 2)
    append_to_curve!(curve, gen_bezier_curve(B, C, 0.6r, 0.6r, BEZIER_SAMPLES)[2:end])
    append_to_curve!(curve, gen_straight_curve(convert(VecE2, C),
                                               convert(VecE2, C+VecE2(0,-params.ver_roadway_length)), 2))
    lane = Lane(LaneTag(length(roadway.segments)+1,1), curve)
    lane.width = params.lane_width
    lane.speed_limit = SPEED_LIMIT
    push!(roadway.segments, RoadSegment(lane.tag.segment, [Lane(lane)]))
    LANE_ID_MAP["right_from_left"] = roadway.segments[1].lanes[1]

    # Append straight right
    curve = gen_straight_curve(convert(VecE2, F+VecE2(params.hor_roadway_length/2,0)),
                               convert(VecE2, F), 2)
    append_to_curve!(curve, gen_straight_curve(convert(VecE2, F), convert(VecE2, A), 2)[2:end])
    append_to_curve!(curve, gen_straight_curve(convert(VecE2, A), convert(VecE2, A+VecE2(-params.hor_roadway_length/2,0)), 2))
    lane = Lane(LaneTag(length(roadway.segments)+1,1), curve)
    lane.width = params.lane_width
    lane.speed_limit = SPEED_LIMIT
    push!(roadway.segments, RoadSegment(lane.tag.segment, [Lane(lane)]))
    LANE_ID_MAP["straight_from_right"] = roadway.segments[2].lanes[1]

    # Append left turn coming from the right
    curve = gen_straight_curve(convert(VecE2, F+VecE2(params.ver_roadway_length,0)), convert(VecE2, F), 2)
    append_to_curve!(curve, gen_bezier_curve(F, C, 0.9r, 0.9r, BEZIER_SAMPLES)[2:end])
    append_to_curve!(curve, gen_straight_curve(convert(VecE2, C), convert(VecE2, C+VecE2(0,-params.ver_roadway_length)), 2))
    lane = Lane(LaneTag(length(roadway.segments)+1,1), curve)
    lane.width = params.lane_width
    lane.speed_limit = SPEED_LIMIT
    push!(roadway.segments, RoadSegment(lane.tag.segment, [Lane(lane)]))
    LANE_ID_MAP["left_from_right"] = roadway.segments[3].lanes[1]

    # Append straight left
    curve = gen_straight_curve(convert(VecE2, B+VecE2(-params.hor_roadway_length/2,0)),
                               convert(VecE2, B), 2)
    append_to_curve!(curve, gen_straight_curve(convert(VecE2, B), convert(VecE2, E), 2)[2:end])
    append_to_curve!(curve, gen_straight_curve(convert(VecE2, E),
                                               convert(VecE2, E+VecE2(params.hor_roadway_length/2,0)), 2))
    lane = Lane(LaneTag(length(roadway.segments)+1,1), curve, width = params.lane_width,speed_limit = SPEED_LIMIT)
    push!(roadway.segments, RoadSegment(lane.tag.segment, [Lane(lane)]))
    LANE_ID_MAP["straight_from_left"] = roadway.segments[4].lanes[1]

    # Append right turn coming from below
    curve = gen_straight_curve(convert(VecE2, D+VecE2(0,-params.ver_roadway_length)), convert(VecE2, D), 2)
    append_to_curve!(curve, gen_bezier_curve(D, E, 0.6r, 0.6r, BEZIER_SAMPLES)[2:end])
    append_to_curve!(curve, gen_straight_curve(convert(VecE2, E), convert(VecE2, E+VecE2(params.ver_roadway_length,0)), 2))
    lane = Lane(LaneTag(length(roadway.segments)+1,1), curve)
    lane.width = params.lane_width
    lane.speed_limit = SPEED_LIMIT
    push!(roadway.segments, RoadSegment(lane.tag.segment, [Lane(lane)]))
    LANE_ID_MAP["ego_right"] = roadway.segments[5].lanes[1]

    # Append left turn coming from below
    curve = gen_straight_curve(convert(VecE2, D+VecE2(0,-params.ver_roadway_length)), convert(VecE2, D), 2)
    append_to_curve!(curve, gen_bezier_curve(D, A, 0.9r, 0.9r, BEZIER_SAMPLES)[2:end])
    append_to_curve!(curve, gen_straight_curve(convert(VecE2, A), convert(VecE2, A+VecE2(-params.hor_roadway_length/2,0)), 2))
    lane = Lane(LaneTag(length(roadway.segments)+1,1), curve)
    lane.width = params.lane_width
    lane.speed_limit = SPEED_LIMIT
    push!(roadway.segments, RoadSegment(lane.tag.segment, [Lane(lane)]))
    LANE_ID_MAP["ego_left"] = roadway.segments[6].lanes[1]
    return roadway, LANE_ID_MAP
end

"""
    append_to_curve!(target::Curve, newstuff::Curve)
add a new lane to the roadway
"""
function append_to_curve!(target::Curve, newstuff::Curve)
    s_end = target[end].s
    for c in newstuff
        push!(target, CurvePt(c.pos, c.s+s_end, c.k, c.kd))
    end
    return target
end

function build_obstacles(params::EnvParams)
    HEIGHT = 6
    WIDTH = 6

    #Left Building
    left_building = ConvexPolygon(4)
    top_right = VecSE2(3*params.lane_width/4, -params.lane_width/2 - 1.5, 0.)
    left_building.pts = [top_right, top_right + VecSE2(-WIDTH, 0., 0.),
                    top_right + VecSE2(-WIDTH, -HEIGHT, 0.), top_right + VecSE2(0., -HEIGHT, 0.)]
    left_building.npts = 4; # a rectangle

    #Right Building
    right_building = ConvexPolygon(4)
    top_right = top_right + VecSE2(top_right.x + 2*params.lane_width + WIDTH, 0., 0.)
    right_building.pts = [top_right, top_right + VecSE2(-WIDTH, 0., 0.),
                    top_right + VecSE2(-WIDTH, -HEIGHT, 0.), top_right + VecSE2(0., -HEIGHT, 0.)]
    right_building.npts = 4; # a rectangle
    return [left_building, right_building]
end


"""
    IntersectionEnv(params::EnvParams)
constructor for the crosswalk environment
"""
function IntersectionEnv(params::EnvParams = EnvParams())
    roadway, lane_id_map = build_t_shape(params)
    obstacles = build_obstacles(params)
    end_pos = round(lane_id_map["ego_left"].curve[end].s)
    return IntersectionEnv(roadway, obstacles, params, lane_id_map, end_pos)
end
