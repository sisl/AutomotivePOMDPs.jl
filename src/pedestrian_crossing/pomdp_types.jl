

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
    ΔT::Float64 = 0.2
    PED_V_NOISE::Vector{Float64} = linspace(-1, 1, 5)
    PED_THETA_NOISE::Vector{Float64} = linspace(-0.78, 0.78, 5)
    
    EGO_Y_MIN::Float64 = -1.
    EGO_Y_MAX::Float64 = 1.
    EGO_Y_RANGE::Vector{Float64} = linspace(EGO_Y_MIN, EGO_Y_MAX, 5)

    EGO_V_MIN::Float64 = 0.
    EGO_V_MAX::Float64 = 14.
    EGO_V_RANGE::Vector{Float64} = linspace(EGO_V_MIN, EGO_V_MAX, 14)

    S_MIN::Float64 = -1.
    S_MAX::Float64 = 49.
    S_RANGE::Vector{Float64} = linspace(S_MIN, S_MAX, 25)

    T_MIN::Float64 = -5.
    T_MAX::Float64 = 5.
    T_RANGE::Vector{Float64} = linspace(T_MIN, T_MAX, 11)

    PED_V_MIN::Float64 = 0.
    PED_V_MAX::Float64 = 2.
    PED_V_RANGE::Vector{Float64} = linspace(PED_V_MIN, PED_V_MAX, 5)

    PED_THETA_MIN::Float64 = 1.57-1.57/2
    PED_THETA_MAX::Float64 = 1.57+1.57/2
    PED_THETA_RANGE::Vector{Float64} = linspace(PED_THETA_MIN, PED_THETA_MAX, 5)


    
    collision_cost::Float64 = -1.0
    action_cost_lon::Float64 = 0.0
    action_cost_lat::Float64 = 0.0
    goal_reward::Float64 = 1.0
    γ::Float64 = 0.95
    
    state_space_grid::GridInterpolations.RectangleGrid = initStateSpace(EGO_Y_RANGE, EGO_V_RANGE, S_RANGE, T_RANGE, PED_THETA_RANGE, PED_V_RANGE)

    state_space_ped::Vector{SingleOCFPedState} = initStateSpacePed(S_RANGE, T_RANGE, PED_THETA_RANGE, PED_V_RANGE)


    state_space::Vector{SingleOCFState} = getStateSpaceVector(state_space_grid)
    action_space::Vector{SingleOCFAction} = initActionSpace(longitudinal_actions, lateral_actions)

    absent::Bool = true
    obstacles::Vector{ConvexPolygon} = []
    ego_vehicle::Vehicle = Vehicle(VehicleState(VecSE2(0.0, 0.0, 0.), 0.0), VehicleDef(), 1)

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



function initStateSpace(EGO_Y_RANGE, EGO_V_RANGE, S_RANGE, T_RANGE, PED_THETA_RANGE, PED_V_RANGE)
    
    return RectangleGrid(EGO_Y_RANGE, EGO_V_RANGE, S_RANGE, T_RANGE, PED_THETA_RANGE, PED_V_RANGE) 
end

function initStateSpacePed(S_RANGE, T_RANGE, PED_THETA_RANGE, PED_V_RANGE)

    ped_grid = RectangleGrid(S_RANGE, T_RANGE, PED_THETA_RANGE, PED_V_RANGE) 
    state_space_ped = SingleOCFPedState[]
    
    for i = 1:length(ped_grid)
        s = ind2x(ped_grid,i)
        push!(state_space_ped,SingleOCFPedState(s[1], s[2], s[3], s[4]))
    end
    # add absent state
    push!(state_space_ped,SingleOCFPedState(-10., -10., 0., 0.))
    return state_space_ped
end


function getStateSpaceVector(grid_space)
    
    state_space = SingleOCFState[]
    
    for i = 1:length(grid_space)
        s = ind2x(grid_space,i)
        push!(state_space,SingleOCFState(s[1], s[2], s[3], s[4], s[5], s[6]))
    end

    # add absent state
    push!(state_space,SingleOCFState(0., 0., -10., -10., 0., 0.))
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

# not nice but faster compared with interpolants
function ego_v_to_state_space_fast(pomdp::SingleOCFPOMDP, ego_v::Float64)

    if ego_v >= maximum(pomdp.EGO_V_RANGE)
        ego_v_dis = pomdp.EGO_V_RANGE[end]
    elseif ego_v <= minimum(pomdp.EGO_V_RANGE)
        ego_v_dis = pomdp.EGO_V_RANGE[1]
    else
        idx = findfirst(x -> x > ego_v, pomdp.EGO_V_RANGE)
        ego_v_dis = pomdp.EGO_V_RANGE[idx]
    end

    return ego_v_dis
end

function ego_v_to_state_space(pomdp::SingleOCFPOMDP, ego_v::Float64)

    id_tmp = interpolants(pomdp.state_space_grid, [pomdp.EGO_Y_MIN, ego_v, pomdp.S_MIN, pomdp.T_MIN, pomdp.PED_THETA_MIN,pomdp.PED_V_MIN])
    id_max = find(a->a==maximum(id_tmp[2]),id_tmp[2])
    return pomdp.state_space[id_tmp[1][id_max]][1].ego_v
end


function ego_y_to_state_space_fast(pomdp::SingleOCFPOMDP, ego_y::Float64)

    if ego_y >= maximum(pomdp.EGO_Y_RANGE)
        return pomdp.EGO_Y_RANGE[end]
    elseif ego_y <= minimum(pomdp.EGO_Y_RANGE)
        return pomdp.EGO_Y_RANGE[1]
    else
        dy = pomdp.EGO_Y_RANGE[2]-pomdp.EGO_Y_RANGE[1]        
        idx = findfirst(x -> x > ego_y, pomdp.EGO_Y_RANGE)
        v1 = pomdp.EGO_Y_RANGE[idx]
        idx = findfirst(x -> x > ego_y-dy, pomdp.EGO_Y_RANGE)
        v2 = pomdp.EGO_Y_RANGE[idx]
 
       if abs(ego_y-v1) < abs(ego_y-v2)
           return v1 
        else
           return v2            
        end
    end
end


function ego_y_to_state_space(pomdp::SingleOCFPOMDP, ego_y::Float64)

    id_tmp = interpolants(pomdp.state_space_grid, [ego_y, pomdp.EGO_V_MIN, pomdp.S_MIN, pomdp.T_MIN, pomdp.PED_THETA_MIN,pomdp.PED_V_MIN])
    id_max = find(a->a==maximum(id_tmp[2]),id_tmp[2])
    return  pomdp.state_space[id_tmp[1][id_max]][1].ego_y
end




