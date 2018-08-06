for COST in 10 15 20 22 24 27 35; do
    nohup julia sarsop_solve.jl -$COST > $COST.out &
done

