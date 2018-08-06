#!/bin/bash

#SBATCH --job-name=sarsop_eval
#SBATCH --nodes 2
#SBATCH --ntasks-per-node 16
#SBATCH --time 2:00:00

module load hdf5
module load julia

julia --machinefile <(srun hostname -s) sarsop_evaluation.jl -$COST
