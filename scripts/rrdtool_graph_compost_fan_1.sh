#/bin/bash

rrdtool graph compost_fan.png -a PNG --alt-autoscale --title="Temperatures de la Cabane" -w 1280 -h 800 -s end-24h -e now \
--watermark "`date`" \
--vertical-label "Temperature (degre C)" \
--right-axis-label "Temperature (degre C)" \
--right-axis-format "%.1lf" \
--right-axis 1:0 \
--no-gridfit \
'DEF:node_00_t_surface=node_00.rrd:t_surface:AVERAGE' \
'DEF:node_00_t_air=node_00.rrd:t_air:AVERAGE' \
'DEF:node_00_humidity=node_00.rrd:humidity:AVERAGE' \
'DEF:node_00_batt_voltage=node_00.rrd:batt_voltage:AVERAGE' \
'DEF:node_02_t_surface=node_02.rrd:t_surface:AVERAGE' \
'DEF:node_02_batt_voltage=node_02.rrd:batt_voltage:AVERAGE' \
'LINE1:node_00_t_surface#FC0303:Node 00 - Temperature de surface \t\n' \
'LINE1:node_00_t_air#FC9E03:Node 00 - Temperature air \t\n' \
'LINE1:node_00_humidity#FC4F03:Node 00 - Humidite air \t\n' \
'LINE1:node_00_batt_voltage#FCA903:Node 00 - Battery voltage \t\n' \
'LINE1:node_02_t_surface#03DAFC:Node 02 - Temperature de surface \t\n' \
'LINE1:node_02_batt_voltage#033CFC:Node 02 - Battery voltage \t\n' \
