#!/bin/bash

#SBATCH --job-name=sarsop
#SBATCH --nodes 1
#SBATCH --ntasks-per-node 1
#SBATCH --time 1:30:00

module load hdf5
module load julia

julia sarsop_solve.jl -$COST
