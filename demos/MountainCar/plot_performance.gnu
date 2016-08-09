set terminal pdf
set output "mountain_car_comparison.pdf"

set yrange[25:300]

set logscale xy

set ylabel "average costs" 
set xlabel "number of training runs"
plot "performance_kbrl.stat" u 1:10 w l lw 3 title "KBRL", "performance_qlearn_grid100x100.stat" u 1:10 w l lw 3 title "Q-Learning with 100x100 grid", 26.971 w l lw 2 title "optimal policy"
