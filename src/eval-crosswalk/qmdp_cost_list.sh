for COST in  1.5; do
  echo "${COST}"
  export COST
  sbatch -o $COST.qmdp_eval qmdp_array.sbatch
done
