for COST in 30; do
    nohup julia -p 40 sarsop_evaluation.jl -$COST > $COST.eval &
done
