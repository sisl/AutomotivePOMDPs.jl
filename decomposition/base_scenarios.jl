# Define several base scenarios derived from the large urban environment

# ego + 2 cars
const TwoCars = UrbanPOMDP(max_cars=2, max_peds=0, obstacles=false)

# ego + 1 ped + 1 car
const PedCar = UrbanPOMDP(max_cars=1, max_peds=1, obstacles=false)

# ego + 1 ped + 1 obstacle
const ObsPed = UrbanPOMDP(max_cars=0, max_peds=1, obstacles=true)

# ego + 1 car + 1 obstacle
const ObsCar = UrbanPOMDP(max_cars=1, max_peds=0, obstacles=true)
