"""
Container type for the environment parameters
"""
@with_kw mutable struct UrbanParams
    # geometry
    x_min::Float64 = -30.0
    x_max::Float64 = 30.0
    y_min::Float64 = -30.0
    inter_x::Float64 = 0.
    inter_y::Float64 = 0.
    inter_width::Float64 = 6.0
    inter_height::Float64 = 1.0
    lane_width::Float64 = 3.0
    nlanes_main::Int64 = 2
    nlanes::Int64 = 1
    # crosswalk
    crosswalk_pos::Vector{VecSE2} = [VecSE2(6, 0., pi/2), VecSE2(-6, 0., pi/2), VecSE2(0., -8., 0.)]
    crosswalk_length::Vector{Float64}  = [20.0, 20., 20.0]
    crosswalk_width::Vector{Float64} = [4.0, 4.0, 3.1]

    stop_line::Float64 = 19. # in m along the ego car lane
    # cars
    speed_limit::Float64 = 10.0 # in m/s
    car_rate::Float64 = 0.3 # probability of a car appearing every time steps
    # peds
    ped_rate::Float64 = 0.3
    ped_max_speed::Float64 = 2.0

end


"""
Urban Environment with intersections and crosswalk
"""
mutable struct UrbanEnv <: OccludedEnv
    roadway::Roadway
    ped_roadway::Roadway
    crosswalks::Vector{Lane}
    obstacles::Vector{ConvexPolygon}
    params::UrbanParams
    priorities::Dict{Tuple{LaneTag, LaneTag}, Bool}
    directions::Dict{Symbol, Tuple{LaneTag, LaneTag}}
end

function UrbanEnv(;params=UrbanParams())
    intersection_params = TInterParams(params.x_min, params.x_max, params.y_min, params.inter_x,
                                       params.inter_y, params.inter_width, params.inter_height,
                                       params. lane_width, params.nlanes_main, params.nlanes,
                                       params. stop_line, params.speed_limit, params.car_rate)
    roadway = gen_T_roadway(intersection_params)
    ped_roadway = Roadway()
    crosswalks = Lane[]
    cw_id = length(roadway.segments)+1
    for i =1:length(params.crosswalk_pos)
        cw_tag = LaneTag(cw_id, 1)

        crosswalk = Lane(cw_tag, gen_straight_curve(VecE2(params.crosswalk_pos[i]) - polar(params.crosswalk_length[i]/2, params.crosswalk_pos[i].θ),
                                                              VecE2(params.crosswalk_pos[i]) + polar(params.crosswalk_length[i]/2, params.crosswalk_pos[i].θ),
                                                               2), width = params.crosswalk_width[i])
        cw_segment = RoadSegment(length(roadway.segments)+1, [crosswalk])
        push!(crosswalks, crosswalk)
        push!(roadway.segments, cw_segment)
        push!(ped_roadway.segments, cw_segment)
        cw_id += 1
    end
    obstacles = Vector{ConvexPolygon}[]
    priorities, directions = priority_map(roadway)
    return UrbanEnv(roadway, ped_roadway, crosswalks, obstacles, params, priorities, directions)
end

function car_roadway(env::UrbanEnv)
    params = env.params
    intersection_params = TInterParams(params.x_min, params.x_max, params.y_min, params.inter_x,
                                    params.inter_y, params.inter_width, params.inter_height,
                                    params. lane_width, params.nlanes_main, params.nlanes,
                                    params. stop_line, params.speed_limit, params.car_rate)
    car_roadway = gen_T_roadway(intersection_params)
    return car_roadway
end
