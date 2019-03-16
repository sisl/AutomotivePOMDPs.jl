var documenterSearchIndex = {"docs": [

{
    "location": "driver_models/#",
    "page": "Driver Models",
    "title": "Driver Models",
    "category": "page",
    "text": ""
},

{
    "location": "driver_models/#AutomotivePOMDPs.ConstantSpeedDawdling",
    "page": "Driver Models",
    "title": "AutomotivePOMDPs.ConstantSpeedDawdling",
    "category": "type",
    "text": "Action type for the pedestrian model\n\n\n\n\n\n"
},

{
    "location": "driver_models/#AutomotivePOMDPs.set_direction!-Tuple{RouteFollowingIDM,Records.Frame{Records.Entity{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64}},AutomotiveDrivingModels.Roadway,Int64}",
    "page": "Driver Models",
    "title": "AutomotivePOMDPs.set_direction!",
    "category": "method",
    "text": "set the lane exit to take at the next junction to reach the goal\n\n\n\n\n\n"
},

{
    "location": "driver_models/#AutomotivePOMDPs.update_priority!-Tuple{CrosswalkDriver,Records.Frame{Records.Entity{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64}},AutomotiveDrivingModels.Roadway,Int64}",
    "page": "Driver Models",
    "title": "AutomotivePOMDPs.update_priority!",
    "category": "method",
    "text": "Check if all the pedestrian have crossed\n\n\n\n\n\n"
},

{
    "location": "driver_models/#AutomotiveDrivingModels.track_longitudinal!-Tuple{RouteFollowingIDM,Float64,Float64,Float64}",
    "page": "Driver Models",
    "title": "AutomotiveDrivingModels.track_longitudinal!",
    "category": "method",
    "text": "Compute acceleration according to IDM\n\n\n\n\n\n"
},

{
    "location": "driver_models/#AutomotivePOMDPs.get_dist_to_end-Tuple{Records.Entity{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64},AutomotiveDrivingModels.Roadway}",
    "page": "Driver Models",
    "title": "AutomotivePOMDPs.get_dist_to_end",
    "category": "method",
    "text": "Return the distance to the end of the lane\n\n\n\n\n\n"
},

{
    "location": "driver_models/#AutomotivePOMDPs.stop_at_dist-Tuple{AutomotiveDrivingModels.DriverModel,AutomotiveDrivingModels.VehicleState,Float64}",
    "page": "Driver Models",
    "title": "AutomotivePOMDPs.stop_at_dist",
    "category": "method",
    "text": "Return the longitudinal acceleration to come to a stop at the distance headway following idm\n\n\n\n\n\n"
},

{
    "location": "driver_models/#AutomotivePOMDPs.stop_at_end-Tuple{AutomotiveDrivingModels.DriverModel,Records.Entity{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64},AutomotiveDrivingModels.Roadway}",
    "page": "Driver Models",
    "title": "AutomotivePOMDPs.stop_at_end",
    "category": "method",
    "text": "Return the longitudinal acceleration to come to a stop at the end of the current lane follows idm style braking\n\n\n\n\n\n"
},

{
    "location": "driver_models/#AutomotivePOMDPs.update_stop!-Tuple{AutomotiveDrivingModels.DriverModel,Records.Entity{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64},AutomotiveDrivingModels.Roadway}",
    "page": "Driver Models",
    "title": "AutomotivePOMDPs.update_stop!",
    "category": "method",
    "text": "set the state stop! to true if veh is stopped at the intersection\n\n\n\n\n\n"
},

{
    "location": "driver_models/#AutomotivePOMDPs.grow_wait_list!-Tuple{StopIntersectionDriver,Records.Frame{Records.Entity{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64}},AutomotiveDrivingModels.Roadway,Int64}",
    "page": "Driver Models",
    "title": "AutomotivePOMDPs.grow_wait_list!",
    "category": "method",
    "text": "If other vehicles reaches the intersection before the ego vehicle they are added to the waitlist\n\n\n\n\n\n"
},

{
    "location": "driver_models/#AutomotivePOMDPs.is_at_intersection-Tuple{StopIntersectionDriver,Records.Entity{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64},AutomotiveDrivingModels.Roadway}",
    "page": "Driver Models",
    "title": "AutomotivePOMDPs.is_at_intersection",
    "category": "method",
    "text": "return true if veh is before the intersection\n\n\n\n\n\n"
},

{
    "location": "driver_models/#AutomotivePOMDPs.tts-Tuple{Float64,Float64}",
    "page": "Driver Models",
    "title": "AutomotivePOMDPs.tts",
    "category": "method",
    "text": "Robust function to compute the time to drive dist at speed v\n\n\n\n\n\n"
},

{
    "location": "driver_models/#AutomotivePOMDPs.ungrow_wait_list!-Tuple{StopIntersectionDriver,Records.Frame{Records.Entity{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64}},AutomotiveDrivingModels.Roadway,Int64}",
    "page": "Driver Models",
    "title": "AutomotivePOMDPs.ungrow_wait_list!",
    "category": "method",
    "text": "Remove from the wait list vehicles that are now exiting the intersection\n\n\n\n\n\n"
},

{
    "location": "driver_models/#AutomotivePOMDPs.update_priority!-Tuple{StopIntersectionDriver,Records.Frame{Records.Entity{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64}},AutomotiveDrivingModels.Roadway,Int64}",
    "page": "Driver Models",
    "title": "AutomotivePOMDPs.update_priority!",
    "category": "method",
    "text": "Check if the ego car has priority or not\n\n\n\n\n\n"
},

{
    "location": "driver_models/#AutomotivePOMDPs.ttc_check-Tuple{TTCIntersectionDriver,Records.Frame{Records.Entity{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64}},AutomotiveDrivingModels.Roadway,Int64}",
    "page": "Driver Models",
    "title": "AutomotivePOMDPs.ttc_check",
    "category": "method",
    "text": "Check if the time to collision is above some threshold\n\n\n\n\n\n"
},

{
    "location": "driver_models/#Driver-Models-1",
    "page": "Driver Models",
    "title": "Driver Models",
    "category": "section",
    "text": "Behavior models for cars and pedestrians.Modules = [AutomotivePOMDPs]\nPages = [\"driver_models/constant_pedestrian.jl\",\n         \"driver_models/crosswalk_driver.jl\",\n         \"driver_models/helpers.jl\",\n         \"driver_models/intelligent_pedestrian_model.jl\",\n         \"driver_models/lidar_sensor.jl\",\n         \"driver_models/old_ttc_driver_file.jl\",\n         \"driver_models/route_following_idm.jl\",\n         \"driver_models/stop.jl\",\n         \"driver_models/stop_intersection_driver.jl\",\n         \"driver_models/ttc_intersection_driver.jl\",\n         \"driver_models/urban_driver.jl\"]"
},

{
    "location": "helpers/#",
    "page": "Helpers",
    "title": "Helpers",
    "category": "page",
    "text": ""
},

{
    "location": "helpers/#AutomotivePOMDPs.get_conflict_lanes-Tuple{AutomotiveDrivingModels.Lane,AutomotiveDrivingModels.Roadway}",
    "page": "Helpers",
    "title": "AutomotivePOMDPs.get_conflict_lanes",
    "category": "method",
    "text": "get_conflict_lanes(crosswalk::Lane, roadway::Roadway)\n\nFind the lanes in roadway that intersects with a crosswalk\n\n\n\n\n\n"
},

{
    "location": "helpers/#AutomotivePOMDPs.get_ego-Tuple{Records.Frame{Records.Entity{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64}}}",
    "page": "Helpers",
    "title": "AutomotivePOMDPs.get_ego",
    "category": "method",
    "text": "return the ego vehicle from a given scene\n\n\n\n\n\n"
},

{
    "location": "helpers/#AutomotivePOMDPs.get_end-Tuple{AutomotiveDrivingModels.Lane}",
    "page": "Helpers",
    "title": "AutomotivePOMDPs.get_end",
    "category": "method",
    "text": "get_end(lane::Lane)\n\nreturn the end longitudinal position of a lane \n\n\n\n\n\n"
},

{
    "location": "helpers/#AutomotivePOMDPs.get_exit_lanes-Tuple{Array{AutomotiveDrivingModels.Lane,1},AutomotiveDrivingModels.Roadway}",
    "page": "Helpers",
    "title": "AutomotivePOMDPs.get_exit_lanes",
    "category": "method",
    "text": "Given a list of lanes returns the list of exits\n\n\n\n\n\n"
},

{
    "location": "helpers/#AutomotivePOMDPs.get_exit_lanes-Tuple{AutomotiveDrivingModels.Roadway}",
    "page": "Helpers",
    "title": "AutomotivePOMDPs.get_exit_lanes",
    "category": "method",
    "text": "return the list of exit lanes, i.e. the one that does not have any exits\n\n\n\n\n\n"
},

{
    "location": "helpers/#AutomotivePOMDPs.get_lane-Tuple{AutomotiveDrivingModels.Roadway,Records.Entity{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64}}",
    "page": "Helpers",
    "title": "AutomotivePOMDPs.get_lane",
    "category": "method",
    "text": "get_lane(roadway::Roadway, vehicle::Vehicle)\nget_lane(roadway::Roadway, vehicle::VehicleState)\n\nreturn the lane where vehicle is in.\n\n\n\n\n\n"
},

{
    "location": "helpers/#AutomotivePOMDPs.get_lanes-Tuple{AutomotiveDrivingModels.Roadway}",
    "page": "Helpers",
    "title": "AutomotivePOMDPs.get_lanes",
    "category": "method",
    "text": "get_lanes(roadway::Roadway)\n\nreturn all the lanes in a roadway that are not crosswalk and not in the ego path \n\n\n\n\n\n"
},

{
    "location": "helpers/#AutomotivePOMDPs.get_start_lanes-Tuple{AutomotiveDrivingModels.Roadway}",
    "page": "Helpers",
    "title": "AutomotivePOMDPs.get_start_lanes",
    "category": "method",
    "text": "return the list of entrance lanes, i.e. the one that does not have any entrances\n\n\n\n\n\n"
},

{
    "location": "helpers/#AutomotivePOMDPs.is_crash-Tuple{Records.Frame{Records.Entity{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64}}}",
    "page": "Helpers",
    "title": "AutomotivePOMDPs.is_crash",
    "category": "method",
    "text": "is_crash(scene::Scene)\n\nreturn true if the ego car is in collision in the given scene, do not check for collisions between other participants\n\n\n\n\n\n"
},

{
    "location": "helpers/#AutomotivePOMDPs.random_route-Tuple{Random.AbstractRNG,AutomotiveDrivingModels.Roadway,AutomotiveDrivingModels.Lane}",
    "page": "Helpers",
    "title": "AutomotivePOMDPs.random_route",
    "category": "method",
    "text": "random_route(rng::AbstractRNG, roadway::Roadway, start_lane::Lane)\n\nGenerate a random route starting from start_lane to a random end node\n\n\n\n\n\n"
},

{
    "location": "helpers/#AutomotivePOMDPs.is_observable_dyna-Tuple{Records.Entity{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64},Records.Entity{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64},Records.Frame{Records.Entity{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64}}}",
    "page": "Helpers",
    "title": "AutomotivePOMDPs.is_observable_dyna",
    "category": "method",
    "text": "is_observable_dyna(ego::Vehicle, car::Vehicle, scene::Scene)\n\nOcclusion checker verifying is car is observable from ego It considers only other vehicles (not pedestrians, not fixed obstacles)\n\n\n\n\n\n"
},

{
    "location": "helpers/#AutomotivePOMDPs.is_observable_fixed-Tuple{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleState,OccludedEnv}",
    "page": "Helpers",
    "title": "AutomotivePOMDPs.is_observable_fixed",
    "category": "method",
    "text": "is_observable_fixed(ego::VehicleState, car::VehicleState, env::OccludedEnv)\nis_observable_fixed(x::Float64, y::Float64, ego::VehicleState, env::OccludedEnv)\n\nOcclusion checker considering only fixed obstacles in the environment:\n\nisobservablefixed(ego::VehicleState, car::VehicleState, env::OccludedEnv) check if car is observable from ego\nisobservablefixed(x::Float64, y::Float64, ego::VehicleState, env::OccludedEnv) check if x, y is observable from ego\n\n\n\n\n\n"
},

{
    "location": "helpers/#AutomotivePOMDPs.collision_checker-Tuple{Records.Entity{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64},Records.Entity{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64}}",
    "page": "Helpers",
    "title": "AutomotivePOMDPs.collision_checker",
    "category": "method",
    "text": "collision_checker(veh_a::Vehicle, veh_b::Vehicle)\ncollision_checker(veh_a::VehicleState, veh_b::VehicleState, veh_a_def::VehicleDef, veh_b_def::VehicleDef)\n\nreturn True if veh_a and veh_b collides. Relies on the parallel axis theorem.\n\n\n\n\n\n"
},

{
    "location": "helpers/#AutomotivePOMDPs.can_push",
    "page": "Helpers",
    "title": "AutomotivePOMDPs.can_push",
    "category": "function",
    "text": "Heuristic to check if a vehicle can be added to a given scene\n\n\n\n\n\n"
},

{
    "location": "helpers/#AutomotivePOMDPs.clean_scene!-Tuple{OccludedEnv,Records.Frame{Records.Entity{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64}}}",
    "page": "Helpers",
    "title": "AutomotivePOMDPs.clean_scene!",
    "category": "method",
    "text": "Remove vehicles that have reached an end scene or that have collided\n\n\n\n\n\n"
},

{
    "location": "helpers/#AutomotivePOMDPs.get_all_lanes-Tuple{AutomotiveDrivingModels.Roadway}",
    "page": "Helpers",
    "title": "AutomotivePOMDPs.get_all_lanes",
    "category": "method",
    "text": "get_all_lanes(roadway::Roadway)\n\nreturns a list of all the lanes present in roadway\n\n\n\n\n\n"
},

{
    "location": "helpers/#AutomotivePOMDPs.get_end_frenet-Tuple{AutomotiveDrivingModels.Roadway,AutomotiveDrivingModels.LaneTag}",
    "page": "Helpers",
    "title": "AutomotivePOMDPs.get_end_frenet",
    "category": "method",
    "text": "Returns a Frenet object of the end of the road Args: Roadway, LaneTag Returns: Frenet\n\n\n\n\n\n"
},

{
    "location": "helpers/#AutomotivePOMDPs.is_crosswalk-Tuple{AutomotiveDrivingModels.Lane}",
    "page": "Helpers",
    "title": "AutomotivePOMDPs.is_crosswalk",
    "category": "method",
    "text": "is_crosswalk(lane::Lane)\n\nreturns true if a lane is a crosswalk\n\n\n\n\n\n"
},

{
    "location": "helpers/#AutomotivePOMDPs.lane_to_segment-Tuple{AutomotiveDrivingModels.Lane,AutomotiveDrivingModels.Roadway}",
    "page": "Helpers",
    "title": "AutomotivePOMDPs.lane_to_segment",
    "category": "method",
    "text": "lane_to_segment(lane::Lane, roadway::Roadway)\n\nConverts a lane to a LineSegment. It only works for straight lanes!\n\n\n\n\n\n"
},

{
    "location": "helpers/#AutomotivePOMDPs.is_observable-Tuple{Records.Entity{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64},Records.Entity{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64},Records.Frame{Records.Entity{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64}},OccludedEnv}",
    "page": "Helpers",
    "title": "AutomotivePOMDPs.is_observable",
    "category": "method",
    "text": "is_observable(ego::Vehicle, car::Vehicle, scene::Scene, env::OccludedEnv)\n\nCheck if car is observable from ego, by considering both fixed and dynamic obstacles\n\n\n\n\n\n"
},

{
    "location": "helpers/#AutomotivePOMDPs.find_separating_axis-Union{Tuple{N}, Tuple{M}, Tuple{SArray{Tuple{M,2},Float64,2,L} where L,SArray{Tuple{N,2},Float64,2,L} where L}} where N where M",
    "page": "Helpers",
    "title": "AutomotivePOMDPs.find_separating_axis",
    "category": "method",
    "text": "find_separating_axis(poly_a::SMatrix{4, 2, Float64}, poly_b::SMatrix{4, 2, Float64})\n\nbuild a list of candidate separating axes from the edges of a   /!\\ edges needs to be ordered\n\n\n\n\n\n"
},

{
    "location": "helpers/#AutomotivePOMDPs.overlap-Union{Tuple{N}, Tuple{M}, Tuple{SArray{Tuple{M,2},Float64,2,L} where L,SArray{Tuple{N,2},Float64,2,L} where L}} where N where M",
    "page": "Helpers",
    "title": "AutomotivePOMDPs.overlap",
    "category": "method",
    "text": "overlap(poly_a::SMatrix{4, 2, Float64}, poly_b::SMatrix{4, 2, Float64})\n\nCheck if two convex polygons overlap, using the parallel axis theorem a polygon is a nx2 matrix where n in the number of verteces http://gamemath.com/2011/09/detecting-whether-two-convex-polygons-overlap/    /!\\ edges needs to be ordered\n\n\n\n\n\n"
},

{
    "location": "helpers/#AutomotivePOMDPs.polygon-Tuple{Vec.VecSE2{Float64},AutomotiveDrivingModels.VehicleDef}",
    "page": "Helpers",
    "title": "AutomotivePOMDPs.polygon",
    "category": "method",
    "text": "polygon(pos::VecSE2{Float64}, veh_def::VehicleDef)\npolygon(x::Float64,y::Float64,theta::Float64,veh_def::VehicleDef)\n\nreturns a 4x2 static matrix corresponding to a rectangle around a car centered at pos and of dimensions specified by veh_def\n\n\n\n\n\n"
},

{
    "location": "helpers/#AutomotivePOMDPs.polygon_projection-Union{Tuple{N}, Tuple{SArray{Tuple{N,2},Float64,2,L} where L,Array{Float64,1}}} where N",
    "page": "Helpers",
    "title": "AutomotivePOMDPs.polygon_projection",
    "category": "method",
    "text": "polygon_projection(poly::SMatrix{4, 2, Float64}, axis::Vector{Float64})\n\nreturn the projection interval for the polygon poly over the axis axis \n\n\n\n\n\n"
},

{
    "location": "helpers/#Helpers-1",
    "page": "Helpers",
    "title": "Helpers",
    "category": "section",
    "text": "Some helpers function including a collision checker and occlusion checker.Modules = [AutomotivePOMDPs]\nPages = [\"utils/helpers.jl\", \"utils/occlusions.jl\", \"utils/collision_checker.jl\", \"utils/rendering.jl\"]"
},

{
    "location": "#",
    "page": "About",
    "title": "About",
    "category": "page",
    "text": ""
},

{
    "location": "#About-1",
    "page": "About",
    "title": "About",
    "category": "section",
    "text": "AutomotivePOMDPs is a collection of driving scenarios modeled as a POMDPs.  The implementations rely on POMDPs.jl  and AutomotiveDrivingModels.jl.  Some models follow the explicit interface and others the generative interface of POMDPs.jl."
},

{
    "location": "scenarios/#",
    "page": "Scenarios",
    "title": "Scenarios",
    "category": "page",
    "text": ""
},

{
    "location": "scenarios/#Scenarios-1",
    "page": "Scenarios",
    "title": "Scenarios",
    "category": "section",
    "text": "module = [AutomotivePOMDPs]\npages = [\"envs/multi_lane_T_env.jl\",\n         \"envs/obstacles.jl\",\n         \"envs/occluded_crosswalk_env.jl\",\n         \"envs/rendering.jl\",\n         \"envs/urban_env.jl\"]"
},

]}
