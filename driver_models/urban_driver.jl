@with_kw mutable struct UrbanDriver <: DriverModel{LonAccelDirection}
    a::LonAccelDirection = LonAccelDirection(0., 1)
    navigator::RouteFollowingIDM = RouteFollowingIDM()
    intersection_driver::IntersectionDriver = IntersectionDriver()
    crosswalk_drivers::Vector{CrosswalkDriver} = [CrosswalkDriver()]


    debug::Bool = false
end

function AutomotiveDrivingModels.reset_hidden_state!(model::UrbanDriver)
    reset_hidden_state!(model.navigator)
    reset_hidden_state!(model.intersection_driver)
    reset_hidden_state!(model.crosswalk_drivers)
    model
end

function AutomotiveDrivingModels.observe!(model::UrbanDriver, scene::Scene, roadway::Roadway, egoid::Int)
    AutomotiveDrivingModels.observe!(model.navigator, scene, roadway, egoid)
    AutomotiveDrivingModels.observe!(model.intersection_driver, scene, roadway, egoid)
    for driver in model.crosswalk_drivers
        AutomotiveDrivingModels.observe!(driver, scene, roadway, egoid)
    end
    a_lon_crosswalks = minimum([driver.a.a_lon for driver in model.crosswalk_drivers])
    # println("intersection driver ", model.intersection_driver.a)
    # println("crosswalk driver ", model.crosswalk_driver.a)
    # take the more conservative of the three actions
    a_lon = min(model.intersection_driver.a.a_lon, a_lon_crosswalks)
    if model.debug
        println("inter ", model.intersection_driver.a.a_lon)
        println("crosswalk 1 ", model.crosswalk_drivers[1].a.a_lon)
    end
    model.a = LonAccelDirection(a_lon, model.navigator.dir)
    return model
end

get_name(::UrbanDriver) = "UrbanDriver"
Base.rand(model::UrbanDriver) = model.a
