#/bin/bash

rrdtool graph compost_fan.png -a PNG --alt-autoscale --title="Temperatures de la Cabane" -w 1280 -h 800 -s end-30d -e now \
--watermark "`date`" \
--vertical-label "Temperature (degre C)" \
--right-axis-label "Temperature (degre C)" \
--right-axis-format "%.1lf" \
--right-axis 1:0 \
--no-gridfit \
'DEF:node_00_batt_voltage=node_00.rrd:batt_voltage:AVERAGE' \
'DEF:node_01_batt_voltage=node_01.rrd:batt_voltage:AVERAGE' \
'DEF:node_02_batt_voltage=node_02.rrd:batt_voltage:AVERAGE' \
'DEF:node_03_batt_voltage=node_03.rrd:batt_voltage:AVERAGE' \
'LINE1:node_00_batt_voltage#FCA903:Node 00 - Battery voltage \t' \
'LINE1:node_01_batt_voltage#8FFC03:Node 01 - Battery voltage \t' \
'LINE1:node_02_batt_voltage#033CFC:Node 02 - Battery voltage \t' \
'LINE1:node_03_batt_voltage#E903FC:Node 03 - Battery voltage \t' 
