#/bin/bash

# Le repertoire qui contiendra les fichiers rrd
#RRA 1 :  au 10 minutes, 8064 enregistrements -> 2 mois 
#RRA 2 :  au 2 heures, 17520 enregistrements -> 4 ans d'enregistrement 
#RRA 3 :  au 6 heures, 14600 enregistrements -> 10 ans d'enregistrement 

#rrd_dir='/mnt/RAMdata/rrd_files/'
rrd_dir='./'



/usr/bin/rrdtool create $rrd_dir/node_00.rrd  --step 600 \
DS:t_1:GAUGE:800:-40:120 \
DS:t_2:GAUGE:800:-40:120 \
DS:t_3:GAUGE:800:-40:120 \
DS:h_1:GAUGE:800:0:120 \
DS:pression:GAUGE:800:0:102000 \
DS:conductivite:GAUGE:800:0:2000 \
DS:batt_voltage:GAUGE:800:0:7 \
DS:txpower:GAUGE:800:5:23 \
DS:last_rssi:GAUGE:800:-125:0 \
RRA:AVERAGE:0.5:1:8064 \
RRA:AVERAGE:0.5:12:17520 \
RRA:AVERAGE:0.1:36:14600 \
RRA:MIN:0.1:36:14600 \
RRA:MAX:0.1:36:14600

/usr/bin/rrdtool create $rrd_dir/node_01.rrd  --step 600 \
DS:t_1:GAUGE:800:-40:120 \
DS:t_2:GAUGE:800:-40:120 \
DS:t_3:GAUGE:800:-40:120 \
DS:h_1:GAUGE:800:0:120 \
DS:pression:GAUGE:800:0:102000 \
DS:conductivite:GAUGE:800:0:2000 \
DS:batt_voltage:GAUGE:800:0:7 \
DS:txpower:GAUGE:800:5:23 \
DS:last_rssi:GAUGE:800:-125:0 \
RRA:AVERAGE:0.5:1:8064 \
RRA:AVERAGE:0.5:12:17520 \
RRA:AVERAGE:0.1:36:14600 \
RRA:MIN:0.1:36:14600 \
RRA:MAX:0.1:36:14600

/usr/bin/rrdtool create $rrd_dir/node_02.rrd  --step 600 \
DS:t_1:GAUGE:800:-40:120 \
DS:t_2:GAUGE:800:-40:120 \
DS:t_3:GAUGE:800:-40:120 \
DS:h_1:GAUGE:800:0:120 \
DS:pression:GAUGE:800:0:102000 \
DS:conductivite:GAUGE:800:0:2000 \
DS:batt_voltage:GAUGE:800:0:7 \
DS:txpower:GAUGE:800:5:23 \
DS:last_rssi:GAUGE:800:-125:0 \
RRA:AVERAGE:0.5:1:8064 \
RRA:AVERAGE:0.5:12:17520 \
RRA:AVERAGE:0.1:36:14600 \
RRA:MIN:0.1:36:14600 \
RRA:MAX:0.1:36:14600

/usr/bin/rrdtool create $rrd_dir/node_03.rrd  --step 600 \
DS:t_1:GAUGE:800:-40:120 \
DS:t_2:GAUGE:800:-40:120 \
DS:t_3:GAUGE:800:-40:120 \
DS:h_1:GAUGE:800:0:120 \
DS:pression:GAUGE:800:0:102000 \
DS:conductivite:GAUGE:800:0:2000 \
DS:batt_voltage:GAUGE:800:0:7 \
DS:txpower:GAUGE:800:5:23 \
DS:last_rssi:GAUGE:800:-125:0 \
RRA:AVERAGE:0.5:1:8064 \
RRA:AVERAGE:0.5:12:17520 \
RRA:AVERAGE:0.1:36:14600 \
RRA:MIN:0.1:36:14600 \
RRA:MAX:0.1:36:14600

/usr/bin/rrdtool create $rrd_dir/node_FE.rrd  --step 600 \
DS:t_avg:GAUGE:800:-40:120 \
DS:t_avg_pc1:GAUGE:800:-40:120 \
DS:t_avg_pc2:GAUGE:800:-40:120 \
DS:t_avg_pc3:GAUGE:800:-40:120 \
DS:t_avg_pc4:GAUGE:800:-40:120 \
DS:pc1:GAUGE:800:-40:120 \
DS:pc2:GAUGE:800:-40:120 \
DS:pc3:GAUGE:800:-40:120 \
DS:pc4:GAUGE:800:-40:120 \
RRA:AVERAGE:0.5:1:8064 \
RRA:AVERAGE:0.5:12:17520 \
RRA:AVERAGE:0.1:36:14600 \
RRA:MIN:0.1:36:14600 \
RRA:MAX:0.1:36:14600
