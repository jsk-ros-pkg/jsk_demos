gnuplot -e  '
set title "pitch";
p "../log/diabolo_system/simulate.log" u 0:1 t "arm" w lp;
rep "../log/diabolo_system/simulate.log" u 0:3 t "pitch" w lp;
pause -1;
'
gnuplot -e  '
set title "yaw";
p "../log/diabolo_system/simulate.log" u 0:2 t "base" w lp;
rep "../log/diabolo_system/simulate.log" u 0:4 t "yaw" w lp;
pause -1;
'
