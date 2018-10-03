

"""
Representation of a state for the SingleOCcludedCrosswalk POMDP,
depends on ADM VehicleState type
"""

#### State type

 struct SingleOCFState
    ego_y::Float64
    ego_v::Float64    
    ped_s::Float64
    ped_T::Float64
    ped_theta::Float64
    ped_v::Float64
end

struct SingleOCFPedState   
    ped_s::Float64
    ped_T::Float64
    ped_theta::Float64
    ped_v::Float64
end

#### Action type

 struct SingleOCFAction
    acc::Float64
    lateral_movement::Float64
end

#### Observaton type

const SingleOCFObs = SingleOCFState




#### POMDP type

@with_kw mutable struct SingleOCFPOMDP <: POMDP{SingleOCFState, SingleOCFAction, SingleOCFObs}
    env::CrosswalkEnv = CrosswalkEnv()
    ego_type::VehicleDef = VehicleDef()
    ped_type::VehicleDef = VehicleDef(AgentClass.PEDESTRIAN, 1.0, 1.0)
    longitudinal_actions::Vector{Float64} = [1.0, 0.0, -1.0, -2.0, -4.0]
    lateral_actions::Vector{Float64} = [1.0, 0.0, -1.0]
    ΔT::Float64 = 0.5
    v_noise::Vector{Float64} = [0] #linspace(-1, 1, 5)
    theta_noise::Vector{Float64} = [0] #linspace(-0.78, 0.78, 5)
    
    pos_obs_noise::Float64 = 1.0
    vel_obs_noise::Float64 = 1.0
    
    
    collision_cost::Float64 = -1.0
    action_cost_lon::Float64 = 0.0
    action_cost_lat::Float64 = 0.0
    goal_reward::Float64 = 1.0
    γ::Float64 = 0.95
    
    state_space_grid::GridInterpolations.RectangleGrid = initStateSpace()


    state_space_ped::Vector{SingleOCFPedState} = initStateSpacePed()

    state_space::Vector{SingleOCFState} = getStateSpaceVector(state_space_grid)
    action_space::Vector{SingleOCFAction} = initActionSpace(longitudinal_actions, lateral_actions)

end


### REWARD MODEL ##################################################################################

function POMDPs.reward(pomdp::SingleOCFPOMDP, s::SingleOCFState, action::SingleOCFAction, sp::SingleOCFState) 
    
    r = 0.
    if collision_checker(pomdp,s)
        r += pomdp.collision_cost
    end
    if s.ped_s < 0
        r += pomdp.goal_reward
    end
    
    if action.acc > 0. && action.acc < 0.0
        r += abs(pomdp.action_cost_lon * action.acc)
    end
        
    if action.lateral_movement > 0 && action.lateral_movement < 0
        r += abs(pomdp.action_cost_lat * action.lateral_movement)
    end
    
    return r
    
end;




### HELPERS

function POMDPs.isterminal(pomdp::SingleOCFPOMDP, s::SingleOCFState)

    if collision_checker(pomdp,s)
        return true
    end
    
    if s.ped_s < 0
        return true
    end 
    
    return false
end


function POMDPs.discount(pomdp::SingleOCFPOMDP)
    return pomdp.γ
end


function AutomotivePOMDPs.collision_checker(pomdp::SingleOCFPOMDP, s::SingleOCFState)
    
    object_a_def = pomdp.ego_type
    object_b_def = pomdp.ped_type
    
    center_a = VecSE2(0, s.ego_y, 0)
    center_b = VecSE2(s.ped_s, s.ped_T, s.ped_theta)
    # first fast check:
    @fastmath begin
        Δ = sqrt((center_a.x - center_b.x)^2 + (center_a.y - center_b.y)^2)
        r_a = sqrt(object_a_def.length*object_a_def.length/4 + object_a_def.width*object_a_def.width/4)
        r_b = sqrt(object_b_def.length*object_b_def.length/4 + object_b_def.width*object_b_def.width/4)
    end
    if Δ ≤ r_a + r_b
        # fast check is true, run parallel axis theorem
        Pa = AutomotivePOMDPs.polygon(center_a, object_a_def)
        Pb = AutomotivePOMDPs.polygon(center_b, object_b_def)
        return AutomotivePOMDPs.overlap(Pa, Pb)
    end
    return false
end



function initStateSpace()
    
    ego_y_min = -1
    ego_y_max = 1
    ego_y_space = linspace(ego_y_min,ego_y_max,5)
   
    ego_v_min = 0
    ego_v_max = 14
    ego_v_space = linspace(ego_v_min,ego_v_max,14)
    
    ped_s_min = -1
    ped_s_max = 49
    ped_s_space = linspace(ped_s_min,ped_s_max,25)
    
    ped_T_min = -5
    ped_T_max = 5
    ped_T_space = linspace(ped_T_min,ped_T_max,10)

    ped_theta_min = 1.57-1.57/2
    ped_theta_max = 1.57+1.57/2
    ped_theta_space = linspace(ped_theta_min,ped_theta_max,5)
    
    ped_v_min = 0
    ped_v_max = 2
    ped_v_space = linspace(ped_v_min,ped_v_max,5)
 
    return RectangleGrid(ego_y_space, ego_v_space, ped_s_space, ped_T_space, ped_theta_space, ped_v_space) 
end

function initStateSpacePed()
    
    ped_s_min = -1
    ped_s_max = 49
    ped_s_space = linspace(ped_s_min,ped_s_max,25)
    
    ped_T_min = -5
    ped_T_max = 5
    ped_T_space = linspace(ped_T_min,ped_T_max,10)

    ped_theta_min = 1.57-1.57/2
    ped_theta_max = 1.57+1.57/2
    ped_theta_space = linspace(ped_theta_min,ped_theta_max,5)
    
    ped_v_min = 0
    ped_v_max = 2
    ped_v_space = linspace(ped_v_min,ped_v_max,5)

    ped_grid = RectangleGrid(ped_s_space, ped_T_space, ped_theta_space, ped_v_space) 
    state_space_ped = SingleOCFPedState[]
    
    for i = 1:length(ped_grid)
        s = ind2x(ped_grid,i)
        push!(state_space_ped,SingleOCFPedState(s[1], s[2], s[3], s[4]))
    end

    return state_space_ped
end

function getStateSpaceVector(grid_space)
    
    state_space = SingleOCFState[]
    
    for i = 1:length(grid_space)
        s = ind2x(grid_space,i)
        push!(state_space,SingleOCFState(s[1], s[2], s[3], s[4], s[5], s[6]))
    end
    
    return state_space
end


function initActionSpace(longitudinal_actions, lateral_actions)
    
  action_space = SingleOCFAction[]

  for lat_a in lateral_actions
    for lon_a in longitudinal_actions
      push!(action_space, SingleOCFAction(lon_a, lat_a))
    end
  end

  return action_space
    
end
