# observation features for the OccludedCrosswalk
# can be either Lidar (defined in ADM)
# or an image representation of the scene

# uncomment for Lidar
function POMDPs.convert_o(::Type{Vector{Float64}}, o::OCObs, pomdp::OCPOMDP)
    return [o.ranges/pomdp.sensor.max_range, o.range_rates/pomdp.env.params.speed_limit]
end



# not used now
mutable struct MapFeatures
    width::Int64
    height::Int64
end
