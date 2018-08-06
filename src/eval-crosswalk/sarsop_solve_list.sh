for COST in 10 15 20 22 25 27 30 35; do
  echo "${COST}"
  export COST
  sbatch -o ${COST}.sarsop sarsop_solve.sbatch
done
