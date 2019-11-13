using AutomotiveDrivingModels


const ROADWAY = gen_straight_roadway(1, 20.0)

function create_vehicle(x::Float64, y::Float64, θ::Float64 = 0.0; id::Int64=1)
    s = VehicleState(VecSE2(x, y, θ), ROADWAY, 0.0)
    return Vehicle(s, VehicleDef(), id)
end

const VEH_REF = create_vehicle(0.0, 0.0, 0.0, id=1)

function no_collisions(xspace, yspace, thetaspace)
    for (x,y,th) in zip(xspace, yspace, thetaspace)
        veh2 = create_vehicle(x,y,th,id=2)
        @assert !collision_checker(VEH_REF, veh2) "Error for veh ", veh2
    end
    return true
end

function collisions(xspace, yspace, thetaspace)
    for (x,y,th) in zip(xspace, yspace, thetaspace)
        veh2 = create_vehicle(x,y,th,id=2)
        @assert collision_checker(VEH_REF, veh2) "Error for veh ", veh2
    end
    return true
end


@testset  "collision checker" begin
    ## Series of test 1: Far field 
    thetaspace = LinRange(0.0, 2*float(pi), 10)
    xspace = LinRange(15.0, 300.0, 10)
    yspace = LinRange(10.0, 300.0, 10)
    @test no_collisions(xspace, yspace, thetaspace)

    ## Series of test 2: Superposition 
    thetaspace = LinRange(0.0, 2*float(pi), 10)
    xspace = LinRange(-2.0, 2.0, 10)
    yspace = LinRange(1.5, 1.5, 10)

    @test collisions(xspace,yspace,thetaspace)
end
