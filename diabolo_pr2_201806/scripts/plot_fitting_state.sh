gnuplot -e  '
set title "pitch";
p "../log/diabolo_system/predict.log" u 0:3 every 5 w lp;
rep "../log/diabolo_system/predict.log" u 0:5 every 5 w lp;
pause -1;
'
gnuplot -e  '
set title "yaw";
p "../log/diabolo_system/predict.log" u 0:4 every 5 w lp;
rep "../log/diabolo_system/predict.log" u 0:6 every 5 w lp;
pause -1;
'
