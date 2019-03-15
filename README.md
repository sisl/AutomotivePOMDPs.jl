# AutomotivePOMDPs: Driving Scenarios formulated as POMDPs

[![Build Status](https://travis-ci.org/sisl/AutomotivePOMDPs.jl.svg?branch=master)](https://travis-ci.org/sisl/AutomotivePOMDPs.jl)
[![Coverage Status](https://coveralls.io/repos/github/sisl/AutomotivePOMDPs.jl/badge.svg?branch=master)](https://coveralls.io/github/sisl/AutomotivePOMDPs.jl?branch=master)

contact: Maxime Bouton, [boutonm@stanford.edu](boutonm@stanford.edu)

This repository consists of different driving scenarios formulated as POMDPs. It provides a generative model for computing policies. A few of them have explicit transition and observation models.

## Installation

To install this package and its dependency run the following in the julia REPL:
```julia
using Pkg
Pkg.add(PackageSpec(url="https://github.com/sisl/Vec.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/Records.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/AutomotiveDrivingModels.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/AutoViz.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/AutoUrban.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/AutomotiveSensors.jl"))
Pkg.add("POMDPs")
using POMDPs
POMDPs.add_registry()
Pkg.add(PackageSpec(url="https://github.com/sisl/AutomotivePOMDPs.jl"))
```
## Code to run

Run `docs/Urban POMDP tutoial.ipynb` for a visualization of the simulation environment.

## Scenarios

This package exports the following POMDP Models:
- `SingleOCPOMDP`: Occluded crosswalk with one single pedestrian. Discrete states and observations with explicit transition and observation model.
- `SingleOIPOMDP`: Occluded intersection with one single car. Discrete states and observations with explicit transition and observation model.
- `OCPOMDP`: Occluded crosswalk with a flow of pedestrian. Generative model implementation with continuous state and observations
- `OIPOMDP`: Occluded T intersection with a flow of cars driving in multiple lanes. Generative model implementation with continuous state and observations
- `UrbanPOMDP` : Occluded T intersection with crosswalks, flow of cars and pedestrians. Generative model implementation with continuous state and observations

These models are defined according to the [POMDPs.jl]() interface. To see how they are parameterized, toggle the documentation using `?` or
use the function `fieldnames` if documentation is not yet written.

Video of the `UrbanPOMDP` scenario:
![urban_lidar](urbanpomdp.gif)

## Sensor Models

- Gaussian noise with ray tracing occlusion checker
- Simple Lidar

## TODOs

- [ ] Move geometry and road topology to AutoUrban
- [ ] Move helpers function to AutomotiveDrivingModels.jl

## Dependencies

- AutomotiveDrivingModels.jl
- POMDPs.jl
