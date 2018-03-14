# AutomotivePOMDPs: Driving Scenarios formulated as POMDPs

contact: Maxime Bouton, [boutonm@stanford.edu](boutonm@stanford.edu)

This repository consists of different driving scenarios formulated as POMDPs. It provides a generative model for computing policies. A few of them have explicit transition and observation models.

## TODOs

- Rename the scenarios, some scenarios have the same type name ` IntersectionEnv`, `CrosswalkEnv`. Reorganise the code structure.
- Move geometry and road topology to AutoUrban

## Dependencies

- AutomotiveDrivingModels.jl
- POMDPs.jl

## Code to run

Run `test.ipynb` for a visualization of the different scenarios.

## Folder structure
