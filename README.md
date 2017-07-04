# ADMScenarios: Scenarios for policy evaluation and adaptive stress testing using AutomotiveDrivingModels.jl

contact: Maxime Bouton, [boutonm@stanford.edu](boutonm@stanford.edu)

This repository consists of different driving scenarios and an evaluation script. The goal is to
quickly evaluate the performance of a decision making policy or use adaptive stress testing.

## Code to run

The notebook evaluation_script_test.ipynb goes through the main functions and offers a vizualisation
of the environment running.


## Folder structure

TODO: re-organize things, have different folders for environment, pedestrian models and sensor models

- eval-env/constant_pedestrian.jl : a simple pedestrian model
- eval-env/pedestrian_flow.jl : some helpers to spawn pedestrian according to a Poisson process (see simulation.jl as well)
- eval-env/simple_sensor.jl : a simple sensor model (gaussian noise)
