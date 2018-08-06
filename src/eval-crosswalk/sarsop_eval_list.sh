for COST in  1 10 20 30 50 60 70 ; do
  echo "${COST}"
  export COST
  sbatch -o ${COST}.sarsop_eval sarsop_array.sbatch
done
