set yrange[-5:105]
set ylabel "% of test runs" 
set xlabel "number of training runs"
plot "performance.stat" u 1:5 w lp title "terminated in goal"
pause -1


set yrange[-5:105]
set ylabel "% of test runs" 
set xlabel "number of training cycles"
plot "performance.stat" u 2:5 w lp title "terminated in goal"
pause -1


set autoscale y

set ylabel "costs" 
set xlabel "number of training runs"
plot "performance.stat" u 1:9 w lp title "average costs"
pause -1
