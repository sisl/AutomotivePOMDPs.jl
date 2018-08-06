#!/bin/bash

#SBATCH --job-name=qmdp_coll
#SBATCH --nodes 2
#SBATCH --ntasks-per-node 16
#SBATCH --time 20:00

module load hdf5
module load julia

julia qmdp_solve.jl -$COST
julia --machinefile <(srun hostname -s) qmdp_evaluation.jl -$COST
