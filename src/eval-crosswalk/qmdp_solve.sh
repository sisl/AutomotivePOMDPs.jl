for COST in 20 ; do 
   nohup julia qmdp_solve.jl -$COST > $COST.qmdp &
done
