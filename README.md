# AutomotivePOMDPs: Driving Scenarios formulated as POMDPs

contact: Maxime Bouton, [boutonm@stanford.edu](boutonm@stanford.edu)

This repository consists of different driving scenarios formulated as POMDPs. It provides a generative model for computing policies. A few of them have explicit transition and observation models.

## Scenarios

This package export the following POMDP Models:
- `SingleOCPOMDP`: Occluded crosswalk with one single pedestrian. Discrete states and observations with explicit transition and observation model.
- `SingleOIPOMDP`: Occluded intersection with one single car. Discrete states and observations with explicit transition and observation model.
- `OCPOMDP`: Occluded crosswalk with a flow of pedestrian. Generative model implementation with continuous state and observations
- `OIPOMDP`: Occluded T intersection with a flow of cars driving in multiple lanes. Generative model implementation with continuous state and observations
- `UrbanPOMDP` : Occluded T intersection with crosswalks, flow of cars and pedestrians. Generative model implementation with continuous state and observations

These models are defined according to the [POMDPs.jl]() interface. To see how they are parameterized, toggle the documentation using `?` or
use the function `fieldnames` if documentation is not yet written.

## Sensor Models

- Gaussian noise with ray tracing occlusion checker
- Simple Lidar

## TODOs

- [ ] Move geometry and road topology to AutoUrban
- [ ] Discretizer: build an approximate explicit POMDP formulation by sampling from the continuous state formulation.

## Dependencies

- AutomotiveDrivingModels.jl
- POMDPs.jl

## Code to run

Run `test.ipynb` for a visualization of the different scenarios.

## Folder structure
