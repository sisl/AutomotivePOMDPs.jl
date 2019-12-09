module AutomotivePOMDPs

using POMDPs
using StatsBase
using Distributions
using POMDPModelTools
using POMDPSimulators
using POMDPPolicies
using BeliefUpdaters
using RLInterface
using Parameters
using GridInterpolations
using StaticArrays
using DiscreteValueIteration
using AutomotiveDrivingModels
using AutoUrban
using AutoViz
using AutomotiveSensors
using Reel
using Random
using DataStructures
using LinearAlgebra
import Cairo

"""
Abstract type to define driving environment with occlusion 
"""
abstract type OccludedEnv end


# helpers
export
        # for rendering
        animate_hist,
        animate_record,
        animate_scenes,

        # helpers
        get_end,
        get_lanes,
        get_start_lanes,
        get_exit_lanes,
        get_ego,
        is_crash,
        direction_from_center,
        random_route,
        is_observable_dyna,
        is_observable_fixed,
        off_the_grid,
        get_conflict_lanes,
        get_colors,
        next_car_id,
        next_ped_id,
        EGO_ID,
        CAR_ID,
        PED_ID,
        EgoDriver

include("constants.jl")
include("utils/helpers.jl")
include("utils/occlusions.jl")
include("utils/rendering.jl")

# envs
export
    OccludedEnv,
    CrosswalkParams,
    CrosswalkEnv,
    TInterParams,
    IntersectionEnv,
    SimpleInterParams,
    SimpleInterEnv,
    gen_T_roadway,
    UrbanParams,
    UrbanEnv,
    ObstacleDistribution,
    sample_obstacles!,
    sample_obstacle!,
    empty_obstacles!,
    add_obstacle!,
    car_roadway

include("envs/occluded_crosswalk_env.jl")
include("envs/multi_lane_T_env.jl")
include("envs/urban_env.jl")
include("envs/obstacles.jl")
include("envs/rendering.jl")

# driver models and action types
export
    action_space,
    get_distribution,
    ConstantPedestrian,
    ConstantSpeedDawdling,
    RouteFollowingIDM,
    set_direction!,
    get_direction,
    LonAccelDirection,
    CrosswalkDriver,
    StopIntersectionDriver,
    TTCIntersectionDriver,
    UrbanDriver,
    IntelligentPedestrian,
    LidarOverlay,
    get_stop_model,
    get_ttc_model

include("driver_models/route_following_idm.jl")
include("driver_models/stop.jl")
include("driver_models/stop_intersection_driver.jl")
include("driver_models/ttc_intersection_driver.jl")
include("driver_models/constant_pedestrian.jl")
include("driver_models/crosswalk_driver.jl")
include("driver_models/urban_driver.jl")
include("driver_models/lidar_sensor.jl")
include("driver_models/intelligent_pedestrian_model.jl")
include("driver_models/helpers.jl")

export
    # pomdp types
    OCPOMDP,
    OCAction,
    OCState,
    OCObs,
    SingleOCPOMDP,
    SingleOCAction,
    SingleOCState,
    SingleOCObs,
    SingleOCBelief,
    SingleOCDistribution,
    SingleOCUpdater,
    OIPOMDP,
    OIAction,
    OIState,
    OIObs,
    SingleOIPOMDP,
    SingleOIAction,
    SingleOIState,
    SingleOIObs,
    state_to_scene,
    UrbanPOMDP,
    UrbanState,
    UrbanAction,
    UrbanObs,
    initial_car,
    initial_pedestrian,
    initial_ego,
    obs_weight,
    rescale!,
    unrescale!,
    obs_to_scene,
    fuse_value,
    fuse_value_min,
    interpolate_state,
    scene_to_states,
    states_to_scene,
    normalized_off_the_grid_pos,
    get_normalized_absent_state,
    split_o,
    n_dims

# single crosswalk
include("explicit_pomdps/single_crosswalk/pomdp_types.jl")
include("explicit_pomdps/single_crosswalk/spaces.jl")
include("explicit_pomdps/single_crosswalk/transition.jl")
include("explicit_pomdps/single_crosswalk/observation.jl")
include("explicit_pomdps/single_crosswalk/belief.jl")
include("explicit_pomdps/single_crosswalk/adm_helpers.jl")
include("explicit_pomdps/single_crosswalk/render_helpers.jl")
include("explicit_pomdps/single_crosswalk/decomposition.jl")

# multi crowsswalk
include("generative_pomdps/multi_crosswalk/pomdp_types.jl")
include("generative_pomdps/multi_crosswalk/generative_model.jl")
include("generative_pomdps/multi_crosswalk/render_helpers.jl")

# single intersection
include("explicit_pomdps/single_intersection/occluded_intersection_env.jl")
include("explicit_pomdps/single_intersection/pomdp_types.jl")
include("explicit_pomdps/single_intersection/spaces.jl")
include("explicit_pomdps/single_intersection/transition.jl")
include("explicit_pomdps/single_intersection/observation.jl")
include("explicit_pomdps/single_intersection/belief.jl")
include("explicit_pomdps/single_intersection/render_helpers.jl")

# multi intersection
include("generative_pomdps/multi_lane_T_intersection/pomdp_types.jl")
include("generative_pomdps/multi_lane_T_intersection/generative_model.jl")
include("generative_pomdps/multi_lane_T_intersection/render_helpers.jl")

#urban
include("generative_pomdps/urban/pomdp_types.jl")
include("generative_pomdps/urban/generative_model.jl")
include("generative_pomdps/urban/render_helpers.jl")

export 
    CarMDP,
    CarMDPState,
    CarMDPAction,
    PedMDP,
    PedMDPState,
    PedMDPAction
export
    labeling,
    get_mdp_state,
    state2scene,
    get_car_vspace,
    get_ped_vspace,
    get_ego_states,
    get_car_states,
    get_ped_states,
    get_car_routes,
    get_ped_lanes,
    ind2ego,
    ind2ped,
    ind2car,
    find_route,
    ego_stateindex,
    ped_stateindex,
    car_stateindex,
    n_ego_states,
    n_car_states,
    n_ped_states,
    get_off_the_grid,
    get_ped_mdp,
    get_car_mdp,
    get_car_models,
    get_stop_model,
    get_ttc_model,
    get_discretized_lane,
    interpolate_pedestrian,
    interpolate_state

# more discrete POMDPs
include("explicit_pomdps/discretization.jl")
include("explicit_pomdps/rendering.jl")
include("explicit_pomdps/car_mdp/pomdp_types.jl")
include("explicit_pomdps/car_mdp/state_space.jl")
include("explicit_pomdps/car_mdp/transition.jl")
include("explicit_pomdps/car_mdp/render_helpers.jl")
include("explicit_pomdps/car_mdp/high_fidelity.jl")
include("explicit_pomdps/pedestrian_mdp/pomdp_types.jl")
include("explicit_pomdps/pedestrian_mdp/state_space.jl")
include("explicit_pomdps/pedestrian_mdp/transition.jl")
include("explicit_pomdps/pedestrian_mdp/render_helpers.jl")
include("explicit_pomdps/pedestrian_mdp/high_fidelity.jl")
# include("explicit_pomdps/pedcar_mdp/driver_models_helpers.jl")
# include("explicit_pomdps/pedcar_mdp/pomdp_types.jl")
# include("explicit_pomdps/pedcar_mdp/state_space.jl")
# include("explicit_pomdps/pedcar_mdp/transition.jl")
# include("explicit_pomdps/pedcar_mdp/high_fidelity.jl")
# include("explicit_pomdps/pedcar_mdp/render_helpers.jl")
include("explicit_pomdps/interpolation.jl")


export
    # decomposition stuff
    TwoCars,
    ObsPed,
    ObsCar,
    TwoCarsScenario,
    PedCarScenario,
    ObsPedScenario,
    ObsCarScenario,
    decompose_input,
    DecomposedPolicy,
    KMarkovDecUpdater,
    KMarkovDecBelief,
    PreviousObsDecUpdater,
    PreviousObsDecBelief,
    initialize_dec_belief,
    BeliefOverlay

include("decomposition/base_scenarios.jl")
include("decomposition/decomposition_wrapper.jl")
include("decomposition/rendering.jl")



end
