'''
Created on 2015-06-22

@author: caribou
'''


class CompostFanData:
    def __init__(self):
        self.node_temperature_surface = 0.0
        self.node_temperature_profondeur = 0.0
        self.node_1_battery_voltage = 0.0
        self.node_2_battery_voltage = 0.0
        self.node_3_battery_voltage = 0.0
        self.node_4_battery_voltage = 0.0
        self.node_1_temperature_ambiant = 0.0
        self.node_1_humidity_ambiant = 0.0
        self.node_1_temperature_surface = 0.0
        self.node_1_temperature_profondeur = 0.0
        self.node_2_temperature_surface = 0.0
        self.node_2_temperature_profondeur = 0.0
        self.node_3_temperature_surface = 0.0
        self.node_3_temperature_profondeur = 0.0
        self.node_4_temperature_surface = 0.0
        self.node_4_temperature_profondeur = 0.0
        self.moyenne_temperature_profondeur = 0.0
        self.moyenne_temperature_surface = 0.0


class GroupeSondeID:
    def __init__(self):
        self.gp1_node_address = 0
        self.gp1_active = 0
        self.gp2_node_address = 0
        self.gp2_active = 0
        self.gp3_node_address = 0
        self.gp3_active = 0
        self.gp4_node_address = 0
        self.gp4_active = 0

class NodeData:
    def __init__(self):
        self.node_id = 0
        self.timestamp = 0
        self.t_1 = 0.0
        self.t_2 = 0.0
        self.t_3 = 0.0
        self.t_4 = 0.0
        self.h_1 = 0.0
        self.pression = 0.0
        self.conductivite = 0
        self.bat_voltage = 0.0
        self.last_rssi = 0
        self.txpower = 0
        self.data_received = 0


class SsrData:
    def __init__(self):
        self.t_avg = 0.0
        self.current_PC = 0
        self.ssr_state = 0


class NodeFanConfig:
    def __init__(self):
        self.PC1 = 0.0
        self.PC2 = 0.0
        self.PC3 = 0.0
        self.PC4 = 0.0
        self.TV1 = 0
        self.TV2 = 0
        self.TV3 = 0
        self.TV4 = 0
        self.TA1 = 0
        self.TA2 = 0
        self.TA3 = 0
        self.TA4 = 0
        self.delais_minute = 0
        self.node_compost_address_0 = 0
        self.node_compost_address_1 = 0
        self.node_compost_address_2 = 0
        self.node_compost_address_3 = 0
        self.node_compost_cfg_0 = 0
        self.node_compost_cfg_1 = 0
        self.node_compost_cfg_2 = 0
        self.node_compost_cfg_3 = 0
        self.node_compost_txt_0 = ''
        self.node_compost_txt_1 = ''
        self.node_compost_txt_2 = ''
        self.node_compost_txt_3 = ''


class CompostNodeData:
    def __init__(self):
        self.node_address = 0
        self.node_cfg = 0
        self.timestamp = 0
        self.ntc_1 = 0.0
        self.ntc_2 = 0.0
        self.bme_humidity = 0.0
        self.bme_temp = 0.0
        self.bme_pression = 0.0
        self.conductivite = 0
        self.batt_voltage = 0.0
        self.delay_minutes = 0
        self.txpower = 0
        self.last_rssi = 0
        self.new_data = 0
        self.clock_ok = 0

class GroupeSonde:
    def __init__(self):
        self.group_id = 0
        self.node_fan_address = 0
        self.nfc = NodeFanConfig()
        self.ssr_data = SsrData()
        self.list_compost_node_data = []
        for x in range(4):
            self.list_compost_node_data.append(CompostNodeData())


class AllNode:
    def __init__(self):
        self.node_0 = NodeData()
        self.node_1 = NodeData()
        self.node_2 = NodeData()
        self.node_3 = NodeData()


class CompostFanConfig:
    def __init__(self):
        self.relais_mode = 0
        self.relais_etat = 0
        self.relais_delais = 0
        self.relais_t_moyenne = 0.0
        self.relais_consigne_temperature_fan = 0.0
        self.relais_consigne_offset_min_temperature_fan = 0.0
        self.relais_consigne_offset_max_temperature_fan = 0.0


class CompostRRDGRAPH:
    def __init__(self):
        self.graph_id = 0
        self.graph_start = ""
        self.graph_end = ""
