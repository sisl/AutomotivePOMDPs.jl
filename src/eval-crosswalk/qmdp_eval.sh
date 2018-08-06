for COST in 1 1.3 1.4 1.5 2 3 4 5 6 7 8 9 10 15 20 ; do 
   nohup julia -p 3 qmdp_evaluation.jl -$COST > $COST.qmdp &
done
