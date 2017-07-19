#!/usr/bin/python

from socket import *
import subprocess
import pickle
import serial
import time
import thread
import csv
import rrdtool
import struct
import os
import datetime
import ConfigParser
import RPi.GPIO as GPIO


from compost_data import *
from fan_compost_config import *

# Global Variables

# Feather Packet
#[HEADER][MESSAGE_TYPE][NODE_ID][DATA_TYPE][DATA_1][DATA_2][DATA_3][DATA_4][END]
# Feather message header
FEATHER_MSG_HEADER = 0xaa
# Feather ending message
FEATHER_MSG_END = 0x55

# Feather message type
FEATHER_MSG_QUERY_DATA = 0x01
FEATHER_MSG_RESPONSE_DATA = 0x02
FEATHER_MSG_SET_DATA = 0x03
FEATHER_MSG_GET_DATA = 0x04
FEATHER_MSG_GET_ALL_DATA = 0x05
FEATHER_MSG_RESPONSE_ALL_DATA = 0x06
FEATHER_MSG_READY_FOR_COMMANDS = 0x07
FEATHER_MSG_REQUEST_ALL_NODE_SLEEP = 0x08
FEATHER_MSG_SEND_ALL_TEMP = 0x09
FEATHER_MSG_ALL_DATA_READY = 0x10
FEATHER_MSG_SET_CLOCK = 0x11
FEATHER_MSG_INCREASE_RF = 0x12
FEATHER_MSG_DECREASE_RF = 0x13
FEATHER_MSG_NODE_READY = 0x14
FEATHER_MSG_SET_NODE_DELAY = 0x15
FEATHER_MSG_SSR_READY = 0x16
FEATHER_MSG_GET_DATETIME = 0x17
FEATHER_MSG_COMPOST_NODE_DATA = 0x18
FEATHER_MSG_SEND_COMPOST_NODE_DATA = 0x19
FEATHER_MSG_SEND_SSR_NODE_CFG = 0x20

# Feather message data type
TEMP_1 = 0x00  # Temperature de surface
TEMP_2 = 0x01  # Temperature de profondeur
TEMP_3 = 0x02  # Temperature ambiante
TEMP_4 = 0x03
HUMIDITY_1 = 0x04  # Humidite ambiante
HUMIDITY_2 = 0x05
OXYGEN_1 = 0x06
CO2_1 = 0x07
TURN_ON_RELAY = 0x08
TURN_OFF_RELAY = 0x09
RELAY_STATE = 0x10
MODE_AUTO = 0x11
READ_BATTERY_VOLTAGE = 0x12
READ_ALL_DATA = 0x13
RELAY_THRESHOLD = 0x14
DELAY_BETWEEN_READS = 0x15
LAST_RSSI = 0x16
SEND_ALL_TEMP = 0x17
SEND_ALL_CFG = 0x18
RELAIS_CFG = 0x19

TCP_STOP_DATA = 0
TCP_GET_NODE_DATA = 1
TCP_GET_NODE_0_DATA = 2
TCP_GET_NODE_1_DATA = 3
TCP_GET_NODE_2_DATA = 4
TCP_GET_NODE_3_DATA = 5
TCP_PUT_RELAIS_CONSIGNE = 6
TCP_PUT_RELAIS_ETAT = 7
TCP_PUT_RELAIS_DELAIS = 8
TCP_GET_RRDGRAPH = 14
TCP_GET_GROUPE_SONDE = 15
TCP_GET_GROUPE_SONDE_CFG_0 = 16
TCP_GET_GROUPE_SONDE_CFG_1 = 17
TCP_GET_GROUPE_SONDE_CFG_2 = 18
TCP_GET_GROUPE_SONDE_CFG_3 = 19
TCP_PUT_RELAIS_CFG_0 = 20
TCP_PUT_RELAIS_CFG_1 = 21
TCP_PUT_RELAIS_CFG_2 = 22
TCP_PUT_RELAIS_CFG_3 = 23
TCP_GET_RELAIS_DATA_0 = 24
TCP_GET_RELAIS_DATA_1 = 25
TCP_GET_RELAIS_DATA_2 = 26
TCP_GET_RELAIS_DATA_3 = 27

os.chdir('/home/pi/CompostFanServer')

led_yellow_pin = 26
led_red_pin = 19


class CompostFAN:
    def __init__(self, no):
        GPIO.setmode(GPIO.BCM)  # Broadcom pin-numbering scheme
        GPIO.setup(led_yellow_pin, GPIO.OUT)  # LED pin set as output
        GPIO.setup(led_red_pin, GPIO.OUT)  # LED pin set as output
        GPIO.output(led_yellow_pin, GPIO.HIGH)
        GPIO.output(led_red_pin, GPIO.HIGH)

        self.gs_id = GroupeSondeID()
        self.config = ConfigParser.ConfigParser()
        self.config.read("compost_fan.ini")
        self.gs_id.gp1_node_address = int(self.config.get("GroupeSonde", "gp1_node_address"))
        self.gs_id.gp2_node_address = int(self.config.get("GroupeSonde", "gp2_node_address"))
        self.gs_id.gp3_node_address = int(self.config.get("GroupeSonde", "gp3_node_address"))
        self.gs_id.gp4_node_address = int(self.config.get("GroupeSonde", "gp4_node_address"))

        print("Groupe Sonde 1 node address : " + str(self.gs_id.gp1_node_address))
        print("Groupe Sonde 2 node address : " + str(self.gs_id.gp2_node_address))
        print("Groupe Sonde 3 node address : " + str(self.gs_id.gp3_node_address))
        print("Groupe Sonde 4 node address : " + str(self.gs_id.gp4_node_address))

        self.no = no
        self.dtt = int(time.time())  # tt - time in seconds
        self.rrd_graph = CompostRRDGRAPH()


        self.nb_groupe_sonde = 0
        self.lgs = []
        for x in range(4):
            self.lgs.append(GroupeSonde())

        self.lgs[0].node_fan_address = self.gs_id.gp1_node_address
        self.lgs[1].node_fan_address = self.gs_id.gp2_node_address
        self.lgs[2].node_fan_address = self.gs_id.gp3_node_address
        self.lgs[3].node_fan_address = self.gs_id.gp4_node_address

#        print("--node address : " + str(self.lgs[0].node_fan_address))

#         self.compost_fan_data = CompostFanData()
#         self.compost_fan_config = CompostFanConfig()
#         self.list_node_data = []
#         for x in range(4):
#             self.list_node_data.append(NodeData())
#         self.ndr = NodeData()
#         self.ndr_00 = NodeData()
#         self.ndr_01 = NodeData()
#         self.ndr_02 = NodeData()
#         self.ndr_03 = NodeData()
#         self.all_node = AllNode()
# #        self.list_node_data[0] = [CompostFanData()]
#         print self.list_node_data[0]
#         print self.list_node_data[1]
#         self.relais_etat = 0
#         self.relais_t_avg = 0.0

        self.node_id = 0
        self.ser = 0
        self.msg_type = 0
        self.data_type = 0
        self.sock = socket(AF_INET, SOCK_STREAM)
        self.socket_data = 0
        self.cnt_serial_data_num = 1
#        self.node_relais_ready_for_command = 0

    def init_serial(self):
        self.ser = serial.Serial()
        self.ser.baudrate = 9600
        self.ser.parity = serial.PARITY_NONE
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.port = serial_feather_port
        self.ser.timeout = 1
        self.ser.open()  # Opens SerialPort
        # print port open or closed

        if self.ser.isOpen():
            print 'Open: ' + self.ser.portstr

    def init_server(self):
        self.sock.bind(('', port))
        self.sock.listen(5)

    def read_float(self, b_bytes):
        b_array_data = bytearray(b_bytes)
        b = ''.join(chr(i) for i in b_array_data)
        import struct
        value_float = struct.unpack('>f', b)
        return value_float[0]

    def read_int8(self, b_bytes):
        b_array_data = bytearray(b_bytes)
        b = ''.join(chr(i) for i in b_array_data)
        import struct
        value_int8 = struct.unpack('>b', b)
        return value_int8[0]

    def read_uint8(self, b_bytes):
        b_array_data = bytearray(b_bytes)
        b = ''.join(chr(i) for i in b_array_data)
        import struct
        value_uint8 = struct.unpack('>B', b)
        return value_uint8[0]

    def read_uint16(self, b_bytes):
        b_array_data = bytearray(b_bytes)
        b = ''.join(chr(i) for i in b_array_data)
        import struct
        value_uint16 = struct.unpack('>H', b)
        return value_uint16[0]

    def read_uint32(self, b_bytes):
        b_array_data = bytearray(b_bytes)
        b = ''.join(chr(i) for i in b_array_data)
        import struct
        value_uint32 = struct.unpack('>I', b)
        return value_uint32[0]

    def handleClient(self, connection):
        print 'handleClient'

        state = 1
        cnt_client_data_num = 0

        while True:
            self.socket_data = connection.recv(1024)
            if not self.socket_data:
                print('no data')
                print
                break

            if self.socket_data == 'STOP_DATA':
                print ('STOP_DATA')
                state = TCP_STOP_DATA
            elif self.socket_data == 'GET_GROUPE_SONDE':
                reply_string = pickle.dumps(self.gs_id)
#                print reply_string
                connection.send(reply_string)
                state = TCP_GET_GROUPE_SONDE
            elif self.socket_data == 'GET_GROUPE_SONDE_CFG_0':
                reply_string = pickle.dumps(self.lgs[0].nfc)
#                print reply_string
                connection.send(reply_string)
                state = TCP_GET_GROUPE_SONDE_CFG_0
            elif self.socket_data == 'GET_GROUPE_SONDE_CFG_1':
                reply_string = pickle.dumps(self.lgs[1].nfc)
#                print reply_string
                connection.send(reply_string)
                state = TCP_GET_GROUPE_SONDE_CFG_1
            elif self.socket_data == 'GET_GROUPE_SONDE_CFG_2':
                reply_string = pickle.dumps(self.lgs[2].nfc)
                # print reply_string
                connection.send(reply_string)
                state = TCP_GET_GROUPE_SONDE_CFG_2
            elif self.socket_data == 'GET_GROUPE_SONDE_CFG_3':
                reply_string = pickle.dumps(self.lgs[3].nfc)
                # print reply_string
                connection.send(reply_string)
                state = TCP_GET_GROUPE_SONDE_CFG_3

            elif self.socket_data == 'GET_RELAIS_DATA_0':
                print 'GET_RELAIS_DATA_0'
                reply_string = pickle.dumps(self.lgs[0].ssr_data)
                # print reply_string
                connection.send(reply_string)
                state = TCP_GET_RELAIS_DATA_0

            elif self.socket_data == 'GET_RELAIS_DATA_1':
                reply_string = pickle.dumps(self.lgs[1].ssr_data)
                # print reply_string
                connection.send(reply_string)
                state = TCP_GET_RELAIS_DATA_1
            elif self.socket_data == 'GET_RELAIS_DATA_2':
                reply_string = pickle.dumps(self.lgs[2].ssr_data)
                # print reply_string
                connection.send(reply_string)
                state = TCP_GET_RELAIS_DATA_2
            elif self.socket_data == 'GET_RELAIS_DATA_3':
                reply_string = pickle.dumps(self.lgs[3].ssr_data)
                # print reply_string
                connection.send(reply_string)
                state = TCP_GET_RELAIS_DATA_3

            elif 'GET_SONDE_1_DATA_' in self.socket_data:
                if self.socket_data[17] == '0':
                    print ('GET_SONDE_1_DATA_0')
                    reply_string = pickle.dumps(self.lgs[0].list_compost_node_data[0])
                    connection.send(reply_string)
                elif self.socket_data[17] == '1':
                    print ('GET_SONDE_1_DATA_1')
                    reply_string = pickle.dumps(self.lgs[1].list_compost_node_data[0])
                    connection.send(reply_string)
                elif self.socket_data[17] == '2':
                    print ('GET_SONDE_1_DATA_2')
                    reply_string = pickle.dumps(self.lgs[2].list_compost_node_data[0])
                    connection.send(reply_string)
                elif self.socket_data[17] == '3':
                    print ('GET_SONDE_1_DATA_3')
                    reply_string = pickle.dumps(self.lgs[3].list_compost_node_data[0])
                    connection.send(reply_string)
            elif 'GET_SONDE_2_DATA_' in self.socket_data:
                if self.socket_data[17] == '0':
                    print ('GET_SONDE_2_DATA_0')
                    reply_string = pickle.dumps(self.lgs[0].list_compost_node_data[1])
                    connection.send(reply_string)
                elif self.socket_data[17] == '1':
                    print ('GET_SONDE_2_DATA_1')
                    reply_string = pickle.dumps(self.lgs[1].list_compost_node_data[1])
                    connection.send(reply_string)
                elif self.socket_data[17] == '2':
                    print ('GET_SONDE_2_DATA_2')
                    reply_string = pickle.dumps(self.lgs[2].list_compost_node_data[1])
                    connection.send(reply_string)
                elif self.socket_data[17] == '3':
                    print ('GET_SONDE_2_DATA_3')
                    reply_string = pickle.dumps(self.lgs[3].list_compost_node_data[1])
                    connection.send(reply_string)
            elif 'GET_SONDE_3_DATA_' in self.socket_data:
                if self.socket_data[17] == '0':
                    print ('GET_SONDE_3_DATA_0')
                    reply_string = pickle.dumps(self.lgs[0].list_compost_node_data[2])
                    connection.send(reply_string)
                elif self.socket_data[17] == '1':
                    print ('GET_SONDE_3_DATA_1')
                    reply_string = pickle.dumps(self.lgs[1].list_compost_node_data[2])
                    connection.send(reply_string)
                elif self.socket_data[17] == '2':
                    print ('GET_SONDE_3_DATA_2')
                    reply_string = pickle.dumps(self.lgs[2].list_compost_node_data[2])
                    connection.send(reply_string)
                elif self.socket_data[17] == '3':
                    print ('GET_SONDE_3_DATA_3')
                    reply_string = pickle.dumps(self.lgs[3].list_compost_node_data[2])
                    connection.send(reply_string)
            elif 'GET_SONDE_4_DATA_' in self.socket_data:
                if self.socket_data[17] == '0':
                    print ('GET_SONDE_4_DATA_0')
                    reply_string = pickle.dumps(self.lgs[0].list_compost_node_data[3])
                    connection.send(reply_string)
                elif self.socket_data[17] == '1':
                    print ('GET_SONDE_4_DATA_1')
                    reply_string = pickle.dumps(self.lgs[1].list_compost_node_data[3])
                    connection.send(reply_string)
                elif self.socket_data[17] == '2':
                    print ('GET_SONDE_4_DATA_2')
                    reply_string = pickle.dumps(self.lgs[2].list_compost_node_data[3])
                    connection.send(reply_string)
                elif self.socket_data[17] == '3':
                    print ('GET_SONDE_4_DATA_3')
                    reply_string = pickle.dumps(self.lgs[3].list_compost_node_data[3])
                    connection.send(reply_string)
            elif self.socket_data == 'GET_NODE_DATA':
#                print ('GET_NODE_DATA')
                reply_string = pickle.dumps(self.compost_fan_data)
                # print reply_string
                connection.send(reply_string)
                state = TCP_GET_NODE_DATA
            elif self.socket_data == 'GET_NODE_0_DATA':
#                print ('GET_NODE_0_DATA')
#                print("Node ID : " + str(self.list_node_data[0].node_id))
#                print("T. 1 : " + str(self.list_node_data[0].t_1))
#                print("T. 2 : " + str(self.list_node_data[0].t_2))
#                print("T. 3 : " + str(self.list_node_data[0].t_3))
#                print("Humidity : " + str(self.list_node_data[0].h_1))
#                print("Battery Voltage: " + str(self.list_node_data[0].bat_voltage))

                reply_string = pickle.dumps(self.list_node_data[0].nfc)
#                print reply_string
                connection.send(reply_string)
                state = TCP_GET_NODE_0_DATA
            elif self.socket_data == 'GET_NODE_1_DATA':
#                print ('GET_NODE_1_DATA')
#                print("Node ID : " + str(self.list_node_data[1].node_id))
#                print("T. 1 : " + str(self.list_node_data[1].t_1))
#                print("T. 2 : " + str(self.list_node_data[1].t_2))
#                print("T. 3 : " + str(self.list_node_data[1].t_3))
#                print("Humidity : " + str(self.list_node_data[1].h_1))
#                print("Battery Voltage: " + str(self.list_node_data[1].bat_voltage))

                reply_string = pickle.dumps(self.list_node_data[1])
#                print reply_string
                connection.send(reply_string)
                state = TCP_GET_NODE_1_DATA
            elif self.socket_data == 'GET_NODE_2_DATA':
#                print ('GET_NODE_2_DATA')
#                print("Node ID : " + str(self.list_node_data[2].node_id))
#                print("T. 1 : " + str(self.list_node_data[2].t_1))
#                print("T. 2 : " + str(self.list_node_data[2].t_2))
#                print("T. 3 : " + str(self.list_node_data[2].t_3))
#                print("Humidity : " + str(self.list_node_data[2].h_1))
#                print("Battery Voltage: " + str(self.list_node_data[2].bat_voltage))

                reply_string = pickle.dumps(self.list_node_data[2])
#                print reply_string
                connection.send(reply_string)
                state = TCP_GET_NODE_2_DATA
            elif self.socket_data == 'GET_NODE_3_DATA':
#                print ('GET_NODE_3_DATA')
#                print("Node ID : " + str(self.list_node_data[3].node_id))
#                print("T. 1 : " + str(self.list_node_data[3].t_1))
#                print("T. 2 : " + str(self.list_node_data[3].t_2))
#                print("T. 3 : " + str(self.list_node_data[3].t_3))
#                print("Humidity : " + str(self.list_node_data[3].h_1))
#                print("Battery Voltage: " + str(self.list_node_data[3].bat_voltage))

                reply_string = pickle.dumps(self.list_node_data[3])
#                print reply_string
                connection.send(reply_string)
                state = TCP_GET_NODE_3_DATA
            elif self.socket_data == 'GET_RELAY_STATE':
                print ('GET_RELAY_STATE')
                reply_string = pickle.dumps(self.compost_fan_config)
                connection.send(reply_string)
            elif self.socket_data == 'PUT_RELAY_CONSIGNE':
                print ('PUT_RELAY_CONSIGNE')
                reply_string = 'Put ready'
                connection.send(reply_string)
                state = TCP_PUT_RELAIS_CONSIGNE
            elif self.socket_data == 'PUT_RELAIS_ETAT':
                print ('PUT_RELAIS_ETAT')
                reply_string = 'Put ready'
                connection.send(reply_string)
                state = TCP_PUT_RELAIS_ETAT
            elif self.socket_data == 'PUT_RELAIS_DELAIS':
                print ('PUT_RELAIS_DELAIS')
                reply_string = 'Put ready'
                connection.send(reply_string)
                state = TCP_PUT_RELAIS_DELAIS
            elif self.socket_data == 'PUT_RELAIS_CFG_0':
                print ('PUT_RELAIS_CFG_0')
                reply_string = 'Put ready'
                connection.send(reply_string)
                state = TCP_PUT_RELAIS_CFG_0
            elif self.socket_data == 'PUT_RELAIS_CFG_1':
                print ('PUT_RELAIS_CFG_1')
                reply_string = 'Put ready'
                connection.send(reply_string)
                state = TCP_PUT_RELAIS_CFG_1
            elif self.socket_data == 'PUT_RELAIS_CFG_2':
                print ('PUT_RELAIS_CFG_2')
                reply_string = 'Put ready'
                connection.send(reply_string)
                state = TCP_PUT_RELAIS_CFG_2
            elif self.socket_data == 'PUT_RELAIS_CFG_3':
                print ('PUT_RELAIS_CFG_3')
                reply_string = 'Put ready'
                connection.send(reply_string)
                state = TCP_PUT_RELAIS_CFG_3

            elif self.socket_data == 'GET_RRDGRAPH':
                reply_string = 'Get ready'
                connection.send(reply_string)
                state = TCP_GET_RRDGRAPH

            else:
                if state == 1:
                    reply = 'Echo=>%s at %s' % (self.socket_data, self.now())
                    print('Server reply : %s' % reply)
                    connection.send(reply.encode())
                elif state == TCP_PUT_RELAIS_CONSIGNE:
                    print ('Receive new consign')
                    self.compost_fan_config = pickle.loads(self.socket_data)
                    print(self.compost_fan_config.relais_consigne_temperature_fan)
                    ba = bytearray(struct.pack('f', self.compost_fan_config.relais_consigne_temperature_fan))
                    print ba[3]
                    print ba[2]
                    print ba[1]
                    print ba[0]
                    b_array_set_message_data = bytearray(9)
                    b_array_set_message_data[0] = FEATHER_MSG_HEADER
                    b_array_set_message_data[1] = FEATHER_MSG_SET_DATA
                    b_array_set_message_data[2] = 254
                    b_array_set_message_data[3] = RELAY_THRESHOLD
                    b_array_set_message_data[4] = ba[3]
                    b_array_set_message_data[5] = ba[2]
                    b_array_set_message_data[6] = ba[1]
                    b_array_set_message_data[7] = ba[0]
                    b_array_set_message_data[8] = FEATHER_MSG_END
                    print('Envoie la consigne...')
                    self.writeserialdata(b_array_set_message_data)
                    state = TCP_STOP_DATA
                    print('Consigne envoye...')
                elif state == TCP_PUT_RELAIS_ETAT:
                    print ('Receive relais etat')
                    self.compost_fan_config = pickle.loads(self.socket_data)

                    b_array_set_message_data = bytearray(9)
                    b_array_set_message_data[0] = FEATHER_MSG_HEADER
                    b_array_set_message_data[1] = FEATHER_MSG_SET_DATA
                    b_array_set_message_data[2] = 254

                    if self.compost_fan_config.relais_etat == 1:
                        b_array_set_message_data[3] = TURN_ON_RELAY
                        print ('relais_on')
                    elif self.compost_fan_config.relais_etat == 0:
                        b_array_set_message_data[3] = TURN_OFF_RELAY
                        print ('relais_off')
                    elif self.compost_fan_config.relais_etat == 2:
                        b_array_set_message_data[3] = MODE_AUTO
                        print ('relais_auto')
                    b_array_set_message_data[4] = FEATHER_MSG_END
                    print('Envoie l etat du relais...')
                    self.writeserialdata(b_array_set_message_data)
                    state = TCP_STOP_DATA
                    print('Etat du relais envoye...')
                elif state == TCP_PUT_RELAIS_DELAIS:
                    print ('Receive new delais')
                    self.compost_fan_config = pickle.loads(self.socket_data)
                    print(self.compost_fan_config.relais_delais)
                    ba = bytearray(struct.pack('B', self.compost_fan_config.relais_delais))
                    print ba[0]
                    b_array_set_message_data = bytearray(6)
                    b_array_set_message_data[0] = FEATHER_MSG_HEADER
                    b_array_set_message_data[1] = FEATHER_MSG_SET_DATA
                    b_array_set_message_data[2] = 254
                    b_array_set_message_data[3] = DELAY_BETWEEN_READS
                    b_array_set_message_data[4] = ba[0]
                    b_array_set_message_data[5] = FEATHER_MSG_END
                    print('Envoie du delais...')
                    self.writeserialdata(b_array_set_message_data)
                    state = TCP_STOP_DATA
                    print('Delais envoye...')
                elif state == TCP_PUT_RELAIS_CFG_0:
                    print ('Receive new relay configuration for GS1')
                    self.set_relais_cfg(0)
                    state = TCP_STOP_DATA
                    print('Nouvelle configuration envoye.')
                elif state == TCP_PUT_RELAIS_CFG_1:
                    print ('Receive new relay configuration for GS1')
                    self.set_relais_cfg(1)
                    state = TCP_STOP_DATA
                    print('Nouvelle configuration envoye.')
                elif state == TCP_PUT_RELAIS_CFG_2:
                    print ('Receive new relay configuration for GS1')
                    self.set_relais_cfg(2)
                    state = TCP_STOP_DATA
                    print('Nouvelle configuration envoye.')
                elif state == TCP_PUT_RELAIS_CFG_3:
                    print ('Receive new relay configuration for GS1')
                    self.set_relais_cfg(3)
                    state = TCP_STOP_DATA
                    print('Nouvelle configuration envoye.')


                elif state == TCP_GET_RRDGRAPH:
                    print ('Receive rrdgraph info')
                    self.rrd_graph = pickle.loads(self.socket_data)
                    print('graph id : ') + str(self.rrd_graph.graph_id)
                    print('graph_start : ') + self.rrd_graph.graph_start
                    print('graph_end : ') + self.rrd_graph.graph_end

                    open_file_string = ('node_' + str(self.rrd_graph.graph_id) + '.png', 'rb')
                    b = os.path.getsize('node_' + str(self.rrd_graph.graph_id) + '.png')
                    print (open_file_string)
                    f_rrdgraph = open('node_' + str(self.rrd_graph.graph_id) + '.png', 'rb')
                    reply_string = str(b)
                    connection.send(reply_string)
                    self.socket_data = connection.recv(1024)
#                    if self.socket_data == "OK":
#                        print ("OK")
                    reply_string = f_rrdgraph.read(1024)
                    ii = 1
                    while reply_string:
#                       print("Sending : " + str(ii))
                        connection.send(reply_string)
                        self.socket_data = connection.recv(1024)
#                        if self.socket_data == "OK":
#                            print ("OK")
                        reply_string = f_rrdgraph.read(1024)
                        ii += 1
                    print("Sending done")
#                    self.socket_data = connection.recv(1024)
#                    if self.socket_data == "OK":
#                        print ("OK")
                    f_rrdgraph.close()
                    state = TCP_STOP_DATA



        print('handleClient finish...')

    def now(self):
        return time.ctime(time.time())

    def writeserialdata(self, serial_data):
        print('writeserialdata')
        self.ser.write(serial_data)         #Writes to the SerialPort

    def read_test(self):
        while 1:
            timestamp = time.strftime('%X')
            line = timestamp
            bytesToRead = self.ser.inWaiting()
            if bytesToRead:
                print ('nombre de byte a lire : ' + str(bytesToRead) )
                b_bytes = self.ser.read(bytesToRead)
                line = line + "  " + ':'.join('{:02x}'.format(ord(c)) for c in b_bytes)
                print line
                print

    def readserialdata(self):
        try:
            print('Start Reading Serial Data...')
            while 1:
                # Lecture du premier byte
                nb_bytes = self.ser.inWaiting()
                if nb_bytes:
    #                print ("Serial Data : nb_byte = " + str(nb_bytes))
                    b_bytes = self.ser.read(1)  # On lit le premier byte
                    nb_bytes = nb_bytes - 1
                    b_array_start_header = bytearray(b_bytes)
                    # verification si le byte correspond a un debut de message
                    if b_array_start_header[0] == FEATHER_MSG_HEADER:
    #                    print ("Serial Data : nb_byte = " + str(nb_bytes))
                        b_bytes = self.ser.read(3)   # Lecture du Message Type, node id et du Data Type
                        nb_bytes = nb_bytes - 3
                        b_array_header = bytearray(b_bytes)
                        timestamp = time.strftime('%d-%m-%Y %X')
                        tt = int(time.time())
                        line = timestamp
                        line = line + "  " + ':'.join('{:02x}'.format(ord(c)) for c in b_bytes)
                        self.msg_type = b_array_header[0]
                        self.node_id = b_array_header[1]
    #                    print ("Serial Data : nb_byte = " + str(nb_bytes))
                        if b_array_header[0] == FEATHER_MSG_RESPONSE_ALL_DATA:
                             if b_array_header[2] == READ_ALL_DATA:
                                 print('FEATHER_MSG_RESPONSE_ALL_DATA')
                                 self.ndr.node_id = self.node_id

                                 self.ndr.timestamp = tt

                                 # Lecture des 4 prochains byte qui correspondent a t_1
                                 b_bytes = self.ser.read(4)   # Lecture du Message Type, node id et du Data Type
                                 b_array_data = bytearray(b_bytes)

                                 b = ''.join(chr(i) for i in b_array_data)
                                 import struct
                                 t1_float = struct.unpack('>f', b)
                                 t1_float = t1_float[0]
                                 self.ndr.t_1 = t1_float

                                 # Lecture des 4 prochains byte qui correspondent a t_2
                                 b_bytes = self.ser.read(4)   # Lecture du Message Type, node id et du Data Type
                                 b_array_data = bytearray(b_bytes)

                                 b = ''.join(chr(i) for i in b_array_data)
                                 import struct
                                 t2_float = struct.unpack('>f', b)
                                 t2_float = t2_float[0]
                                 self.ndr.t_2 = t2_float

                                 # Lecture des 4 prochains byte qui correspondent a t_3
                                 b_bytes = self.ser.read(4)   # Lecture du Message Type, node id et du Data Type
                                 b_array_data = bytearray(b_bytes)

    #                             if self.node_id == 0x00:
                                 b = ''.join(chr(i) for i in b_array_data)
                                 import struct
                                 t3_float = struct.unpack('>f', b)
                                 t3_float = t3_float[0]
                                 self.ndr.t_3 = t3_float
    #                            else:
      #                               self.ndr.t_3 = -40

                                 # Lecture des 4 prochains byte qui correspondent a h_1
                                 b_bytes = self.ser.read(4)   # Lecture du Message Type, node id et du Data Type
                                 b_array_data = bytearray(b_bytes)

      #                           if self.node_id == 0x00:
                                 b = ''.join(chr(i) for i in b_array_data)
                                 import struct
                                 h1_float = struct.unpack('>f', b)
                                 h1_float = h1_float[0]
                                 self.ndr.h_1 = h1_float
       #                      else:
       #                              self.ndr.h_1 = 0.0

                                 # Lecture des 4 prochains byte qui correspondent a t_2
                                 b_bytes = self.ser.read(4)   # Lecture du Message Type, node id et du Data Type
                                 b_array_data = bytearray(b_bytes)

                                 b = ''.join(chr(i) for i in b_array_data)
                                 import struct
                                 batt_float = struct.unpack('>f', b)
                                 batt_float = batt_float[0]
                                 self.ndr.bat_voltage = batt_float


                                 if self.ndr.node_id == 0:
                                     # self.list_node[0] = self.ndr
                                     self.all_node.node_0 = self.ndr
                                     self.ndr_00 = self.ndr
                                     print('NODE 0')
                                     print("Node ID : " + str(self.all_node.node_0.node_id))
                                     print("T. 1 : " + str(self.all_node.node_0.t_1))
                                     print("T. 2 : " + str(self.all_node.node_0.t_2))
                                     print("T. 3 : " + str(self.all_node.node_0.t_3))
                                     print("Humidity : " + str(self.all_node.node_0.h_1))
                                     print("Battery Voltage: " + str(self.all_node.node_0.bat_voltage))
                                 elif self.ndr.node_id == 1:
                                     self.all_node.node_1 = self.ndr
                                     # self.list_node[1] = self.ndr
                                     self.ndr_01 = self.ndr
                                     print('NODE 1')
                                     print("Node ID : " + str(self.all_node.node_1.node_id))
                                     print("T. 1 : " + str(self.all_node.node_1.t_1))
                                     print("T. 2 : " + str(self.all_node.node_1.t_2))
                                     print("T. 3 : " + str(self.all_node.node_1.t_3))
                                     print("Humidity : " + str(self.all_node.node_1.h_1))
                                     print("Battery Voltage: " + str(self.all_node.node_1.bat_voltage))
                                 elif self.ndr.node_id == 2:
                                     self.all_node.node_2 = self.ndr
                                     # self.list_node[2] = self.ndr
                                     self.ndr_02 = self.ndr
                                     print('NODE 2')
                                     print("Node ID : " + str(self.all_node.node_2.node_id))
                                     print("T. 1 : " + str(self.all_node.node_2.t_1))
                                     print("T. 2 : " + str(self.all_node.node_2.t_2))
                                     print("T. 3 : " + str(self.all_node.node_2.t_3))
                                     print("Humidity : " + str(self.all_node.node_2.h_1))
                                     print("Battery Voltage: " + str(self.all_node.node_2.bat_voltage))
                                 elif self.ndr.node_id == 3:
                                     self.all_node.node_3 = self.ndr
                                     # self.list_node[3] = self.ndr
                                     self.ndr_03 = self.ndr
                                     print('NODE 3')
                                     print("Node ID : " + str(self.all_node.node_3.node_id))
                                     print("T. 1 : " + str(self.all_node.node_3.t_1))
                                     print("T. 2 : " + str(self.all_node.node_3.t_2))
                                     print("T. 3 : " + str(self.all_node.node_3.t_3))
                                     print("Humidity : " + str(self.all_node.node_3.h_1))
                                     print("Battery Voltage: " + str(self.all_node.node_3.bat_voltage))

                                 #print self.cnt_serial_data_num
                                 print tt
    #                            cmd = []
    #                            cmd.append('/usr/bin/rrdtool')
    #                            cmd.append('update')
    #                            cmd.append(rrd_dir + 'node_' + '{:02d}'.format(self.ndr.node_id) + '.rrd')
    #                            if self.node_id == 0x00:
    #                                cmd.extend(['-t', "t_surface:t_profondeur:t_air:humidity:batt_voltage"])
    #                                cmd.append(str(tt) + ':' + str(self.ndr.t_1) +
    #                                           ":" + str(self.ndr.t_2) +
    #                                           ":" + str(self.ndr.t_3) + ":" + str(self.ndr.h_1) +
    #                                           ":" + str(self.ndr.bat_voltage))
    #                            else:
    #                                cmd.extend(['-t', "t_surface:t_profondeur:batt_voltage"])
    #                                cmd.append(str(tt) + ':' + str(self.ndr.t_1) +
    #                                           ":" + str(self.ndr.t_2) +
    #                                           ":" + str(self.ndr.bat_voltage))

                                 # now execute the command
                                 # this could really do with having some error-trapping
    #                           subprocess.call(cmd)
                        elif b_array_header[0] == FEATHER_MSG_READY_FOR_COMMANDS:
                            print ('FEATHER_MSG_READY_FOR_COMMANDS : Node : ' + str(self.node_id))
    #                        self.node_relais_ready_for_command = 1
                            print ('Feather relay id : ' + str(self.node_id))
                            if self.gs_id.gp1_node_address == self.node_id:
                                print("Groupe 1 actif")
                                self.gs_id.gp1_active = 1
                            elif self.gs_id.gp2_node_address == self.node_id:
                                print("Groupe 2 actif")
                                self.gs_id.gp2_active = 1
                            elif self.gs_id.gp3_node_address == self.node_id:
                                print("Groupe 3 actif")
                                self.gs_id.gp3_active = 1
                            elif self.gs_id.gp4_node_address == self.node_id:
                                print("Groupe 4 actif")
                                self.gs_id.gp4_active = 1

    #                        read_flush_bytes = self.ser.read(250)
    #                        print ("read_flush_bytes : " + read_flush_bytes)
                            b_array_set_message_data = bytearray(5)
                            b_array_set_message_data[0] = FEATHER_MSG_HEADER
                            b_array_set_message_data[1] = FEATHER_MSG_SEND_SSR_NODE_CFG
                            b_array_set_message_data[2] = self.node_id
                            b_array_set_message_data[3] = SEND_ALL_CFG
                            b_array_set_message_data[4] = FEATHER_MSG_END
                            print('Envoie la commande SEND_ALL_CFG')
                            self.writeserialdata(b_array_set_message_data)
                        elif b_array_header[0] == FEATHER_MSG_GET_DATETIME:
                            print('Envoie la date et heure')
                            self.set_relais_date_time(b_array_header[1])
                        elif b_array_header[0] == FEATHER_MSG_GET_ALL_DATA:
                            print ("FEATHER_MSG_GET_ALL_DATA : Node : " + str(self.node_id))
                        elif b_array_header[0] == FEATHER_MSG_RESPONSE_DATA:
                            print ('FEATHER_MSG_RESPONSE_DATA from node id : ' + str(self.node_id))
                            if b_array_header[2] == SEND_ALL_TEMP:
                                # Node 0
                                cur_groupe_sonde = self.get_groupe_sonde_id(self.node_id)
                                print('lecture des donnes pour le groupe sonde : ' + str(cur_groupe_sonde))
                                for x in range(0, 4):
                                    # Lecture de node_address
                                    b_bytes = self.ser.read(1)
                                    temp_data = self.read_uint8(b_bytes)
                                    self.lgs[cur_groupe_sonde].list_compost_node_data[x].node_address = temp_data
                                    print ('Node ' + str(x) + ' : node_address : '
                                           + str(self.lgs[cur_groupe_sonde].list_compost_node_data[x].node_address))

                                    # Lecture de node_cfg
                                    b_bytes = self.ser.read(1)
                                    temp_data = self.read_uint8(b_bytes)
                                    self.lgs[cur_groupe_sonde].list_compost_node_data[x].node_cfg = temp_data
                                    print ('Node ' + str(x) + ' : node_cfg : '
                                           + str(self.lgs[cur_groupe_sonde].list_compost_node_data[x].node_cfg))

                                    # Lecture du timestamp
                                    b_bytes = self.ser.read(4)
                                    temp_data = self.read_uint32(b_bytes)
                                    self.lgs[cur_groupe_sonde].list_compost_node_data[x].timestamp = temp_data
                                    print ('Node ' + str(x) + ' : timestamp : '
                                           + str(self.lgs[cur_groupe_sonde].list_compost_node_data[x].timestamp))

                                    # Lecture de ntc_1
                                    b_bytes = self.ser.read(4)
                                    temp_data = self.read_float(b_bytes)
                                    self.lgs[cur_groupe_sonde].list_compost_node_data[x].ntc_1 = temp_data
                                    print ('Node ' + str(x) + ' : ntc_1 : '
                                           + str(self.lgs[cur_groupe_sonde].list_compost_node_data[x].ntc_1))

                                    # Lecture de ntc_2
                                    b_bytes = self.ser.read(4)
                                    temp_data = self.read_float(b_bytes)
                                    self.lgs[cur_groupe_sonde].list_compost_node_data[x].ntc_2 = temp_data
                                    print ('Node ' + str(x) + ' : ntc_2 : '
                                           + str(self.lgs[cur_groupe_sonde].list_compost_node_data[x].ntc_2))

                                    # Lecture de bme_humidity
                                    b_bytes = self.ser.read(4)
                                    temp_data = self.read_float(b_bytes)
                                    self.lgs[cur_groupe_sonde].list_compost_node_data[x].bme_humidity = temp_data
                                    print ('Node ' + str(x) + ' : bme_humidity : '
                                           + str(self.lgs[cur_groupe_sonde].list_compost_node_data[x].bme_humidity))

                                    # Lecture de bme_temp
                                    b_bytes = self.ser.read(4)
                                    temp_data = self.read_float(b_bytes)
                                    self.lgs[cur_groupe_sonde].list_compost_node_data[x].bme_temp = temp_data
                                    print ('Node ' + str(x) + ' : bme_temp : '
                                           + str(self.lgs[cur_groupe_sonde].list_compost_node_data[x].bme_temp))

                                    # Lecture de bme_pression
                                    b_bytes = self.ser.read(4)
                                    temp_data = self.read_float(b_bytes)
                                    self.lgs[cur_groupe_sonde].list_compost_node_data[x].bme_pression = temp_data
                                    print ('Node ' + str(x) + ' : bme_pression : '
                                           + str(self.lgs[cur_groupe_sonde].list_compost_node_data[x].bme_pression))

                                    # Lecture de conductivite
                                    b_bytes = self.ser.read(2)
                                    temp_data = self.read_uint16(b_bytes)
                                    self.lgs[cur_groupe_sonde].list_compost_node_data[x].conductivite = temp_data
                                    print ('Node ' + str(x) + ' : conductivite : '
                                           + str(self.lgs[cur_groupe_sonde].list_compost_node_data[x].conductivite))

                                    # Lecture de batt_voltage
                                    b_bytes = self.ser.read(4)
                                    temp_data = self.read_float(b_bytes)
                                    self.lgs[cur_groupe_sonde].list_compost_node_data[x].batt_voltage = temp_data
                                    print ('Node ' + str(x) + ' : batt_voltage : '
                                           + str(self.lgs[cur_groupe_sonde].list_compost_node_data[x].batt_voltage))

                                    # Lecture de delay_minutes
                                    b_bytes = self.ser.read(1)
                                    temp_data = self.read_uint8(b_bytes)
                                    self.lgs[cur_groupe_sonde].list_compost_node_data[x].delay_minutes = temp_data
                                    print ('Node ' + str(x) + ' : delay_minutes : '
                                           + str(self.lgs[cur_groupe_sonde].list_compost_node_data[x].delay_minutes))

                                    # Lecture de txpower
                                    b_bytes = self.ser.read(1)
                                    temp_data = self.read_uint8(b_bytes)
                                    self.lgs[cur_groupe_sonde].list_compost_node_data[x].txpower = temp_data
                                    print ('Node ' + str(x) + ' : txpower : '
                                           + str(self.lgs[cur_groupe_sonde].list_compost_node_data[x].txpower))

                                    # Lecture de last_rssi
                                    b_bytes = self.ser.read(1)
                                    temp_data = self.read_int8(b_bytes)
                                    self.lgs[cur_groupe_sonde].list_compost_node_data[x].last_rssi = temp_data
                                    print ('Node ' + str(x) + ' : last_rssi : '
                                           + str(self.lgs[cur_groupe_sonde].list_compost_node_data[x].last_rssi))

                                    # Lecture de new_data
                                    b_bytes = self.ser.read(1)
                                    temp_data = self.read_uint8(b_bytes)
                                    self.lgs[cur_groupe_sonde].list_compost_node_data[x].new_data = temp_data
                                    print ('Node ' + str(x) + ' : new_data : '
                                           + str(self.lgs[cur_groupe_sonde].list_compost_node_data[x].new_data))

                                    # Lecture de clock_ok
                                    b_bytes = self.ser.read(1)
                                    temp_data = self.read_uint8(b_bytes)
                                    self.lgs[cur_groupe_sonde].list_compost_node_data[x].clock_ok = temp_data
                                    print ('Node ' + str(x) + ' : clock_ok : '
                                           + str(self.lgs[cur_groupe_sonde].list_compost_node_data[x].clock_ok))

                                # Lecture du timestamp du relais
                                b_bytes = self.ser.read(4)
                                temp_data = self.read_uint32(b_bytes)
                                self.lgs[cur_groupe_sonde].ssr_data.timestamp = temp_data
                                print ('timestamp_relais :' + str(self.lgs[cur_groupe_sonde].ssr_data.timestamp))

                                # Lecture de t_avg
                                b_bytes = self.ser.read(4)
                                temp_data = self.read_float(b_bytes)
                                self.lgs[cur_groupe_sonde].ssr_data.t_avg = temp_data
                                print ('t_avg :' + str(self.lgs[cur_groupe_sonde].ssr_data.t_avg))

                                # Lecture de current_PC
                                b_bytes = self.ser.read(1)
                                temp_data = self.read_uint8(b_bytes)
                                self.lgs[cur_groupe_sonde].ssr_data.current_PC = temp_data
                                print ('current_PC : ' + str(self.lgs[cur_groupe_sonde].ssr_data.current_PC))

                                # Lecture de ssr_state
                                b_bytes = self.ser.read(1)
                                temp_data = self.read_uint8(b_bytes)
                                self.lgs[cur_groupe_sonde].ssr_data.ssr_state = temp_data
                                print ('ssr_state : ' + str(self.lgs[cur_groupe_sonde].ssr_data.ssr_state))

                                b_bytes = self.ser.read(1)
                                print(b_bytes)



                                # Ecriture des donnes dans le fichier CSV
                                now = datetime.datetime.now()
                                date_now = now.strftime("%m-%Y")
                                sonde_1_timestamp = self.lgs[cur_groupe_sonde].list_compost_node_data[0].timestamp
                                sonde_1_datetime = datetime.datetime.fromtimestamp(sonde_1_timestamp)
                                sonde_2_timestamp = self.lgs[cur_groupe_sonde].list_compost_node_data[1].timestamp
                                sonde_2_datetime = datetime.datetime.fromtimestamp(sonde_2_timestamp)
                                sonde_3_timestamp = self.lgs[cur_groupe_sonde].list_compost_node_data[2].timestamp
                                sonde_3_datetime = datetime.datetime.fromtimestamp(sonde_3_timestamp)
                                sonde_4_timestamp = self.lgs[cur_groupe_sonde].list_compost_node_data[3].timestamp
                                sonde_4_datetime = datetime.datetime.fromtimestamp(sonde_4_timestamp)
                                relais_timestamp = self.lgs[cur_groupe_sonde].ssr_data.timestamp
                                relais_datetime = datetime.datetime.fromtimestamp(relais_timestamp)

                                if os.path.exists(compost_csv_directory + str(cur_groupe_sonde+1) + '/'):
                                    with open(compost_csv_directory + str(cur_groupe_sonde+1) + '/' + compost_csv_filename + '_' + date_now + '.csv', 'ab') as csvfile:
                                        compost_data_writer = csv.writer(csvfile, delimiter=',', quoting=csv.QUOTE_MINIMAL)
                                        compost_data_writer.writerow([self.lgs[cur_groupe_sonde].node_fan_address,
                                                                    relais_datetime.strftime("%Y-%m-%d %H:%M:%S"),
                                                                      "{0:.2f}".format(self.lgs[cur_groupe_sonde].ssr_data.t_avg),
                                                                      '{:02d}'.format(self.lgs[cur_groupe_sonde].ssr_data.current_PC),
                                                                      '{:02d}'.format(self.lgs[cur_groupe_sonde].ssr_data.ssr_state),
                                                                      self.lgs[cur_groupe_sonde].nfc.node_compost_txt_0,
                                                                        sonde_1_datetime.strftime("%Y-%m-%d %H:%M:%S"),
                                                                      '{:02d}'.format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              0].node_address),
                                                                      "{0:.2f}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              0].ntc_1),
                                                                      "{0:.2f}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              0].ntc_2),
                                                                      "{0:.2f}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              0].bme_temp),
                                                                      "{0:.2f}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              0].bme_humidity),
                                                                      "{0:.2f}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              0].bme_pression),
                                                                      "{0:02d}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              0].conductivite),
                                                                      "{0:.2f}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              0].batt_voltage),
                                                                      "{:02d}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              0].last_rssi),
                                                                      "{:02d}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              0].txpower),
                                                                      "{:02d}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              0].new_data),
                                                                      self.lgs[cur_groupe_sonde].nfc.node_compost_txt_1,
                                                                        sonde_2_datetime.strftime("%Y-%m-%d %H:%M:%S"),
                                                                      '{:02d}'.format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              1].node_address),
                                                                      "{0:.2f}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              1].ntc_1),
                                                                      "{0:.2f}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              1].ntc_2),
                                                                      "{0:.2f}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              1].bme_temp),
                                                                      "{0:.2f}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              1].bme_humidity),
                                                                      "{0:.2f}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              1].bme_pression),
                                                                      "{0:02d}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              1].conductivite),
                                                                      "{0:.2f}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              1].batt_voltage),
                                                                      "{:02d}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              1].last_rssi),
                                                                      "{:02d}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              1].txpower),
                                                                      "{:02d}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              1].new_data),
                                                                      self.lgs[cur_groupe_sonde].nfc.node_compost_txt_2,
                                                                     sonde_3_datetime.strftime("%Y-%m-%d %H:%M:%S"),
                                                                      '{:02d}'.format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              2].node_address),
                                                                      "{0:.2f}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              2].ntc_1),
                                                                      "{0:.2f}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              2].ntc_2),
                                                                      "{0:.2f}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              2].bme_temp),
                                                                      "{0:.2f}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              2].bme_humidity),
                                                                      "{0:.2f}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              2].bme_pression),
                                                                      "{0:02d}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              2].conductivite),
                                                                      "{0:.2f}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              2].batt_voltage),
                                                                      "{:02d}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              2].last_rssi),
                                                                      "{:02d}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              2].txpower),
                                                                      "{:02d}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              2].new_data),
                                                                      self.lgs[cur_groupe_sonde].nfc.node_compost_txt_3,
                                                                        sonde_4_datetime.strftime("%Y-%m-%d %H:%M:%S"),
                                                                      '{:02d}'.format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              3].node_address),
                                                                      "{0:.2f}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              3].ntc_1),
                                                                      "{0:.2f}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              3].ntc_2),
                                                                      "{0:.2f}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              3].bme_temp),
                                                                      "{0:.2f}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              3].bme_humidity),
                                                                      "{0:.2f}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              3].bme_pression),
                                                                      "{0:02d}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              3].conductivite),
                                                                      "{0:.2f}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              3].batt_voltage),
                                                                      "{:02d}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              3].last_rssi),
                                                                      "{:02d}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              3].txpower),
                                                                      "{:02d}".format(
                                                                          self.lgs[cur_groupe_sonde].list_compost_node_data[
                                                                              3].new_data)]),
                                else:
                                    print ("Ne peut pas ouvrir le fichier : " + compost_csv_directory + str(cur_groupe_sonde+1) + '/')
                                # Ecriture des donnees dans un fichier rrd
                                #Ecriture des donnees des noeuds de compost
    #                             for x in range(4):
    #                                 if self.list_node_data[x].data_received:
    #                                     cmd = []
    #                                     cmd.append('/usr/bin/rrdtool')
    #                                     cmd.append('update')
    #                                     cmd.append(rrd_dir + 'node_' + '{:02d}'.format(x) + '.rrd')
    #                                     cmd.extend(['-t', "t_1:t_2:t_3:h_1:pression:conductivite:batt_voltage:txpower:last_rssi"])
    #                                     cmd.append(str(tt) + ':' + str(self.list_node_data[x].t_1) +
    #                                                          ":" + str(self.list_node_data[x].t_2) +
    #                                                          ":" + str(self.list_node_data[x].t_3) +
    #                                                          ":" + str(self.list_node_data[x].h_1) +
    #                                                          ":" + str(self.list_node_data[x].pression) +
    #                                                          ":" + str(self.list_node_data[x].conductivite) +
    #                                                          ":" + str(self.list_node_data[x].bat_voltage) +
    #                                                          ":" + str(self.list_node_data[x].txpower) +
    #                                                          ":" + str(self.list_node_data[x].last_rssi))
    #
    #                                     # now execute the command
    #                                     # this could really do with having some error-trapping
    # #                                   print(cmd)
    #                                     subprocess.call(cmd)
    #                             # Ecriture des donnes du relais
    #                             cmd = []
    #                             cmd.append('/usr/bin/rrdtool')
    #                             cmd.append('update')
    #                             cmd.append(rrd_dir + 'node_' + 'FE' + '.rrd')
    #                             cmd.extend(['-t', "setpoint_relais:etat_relais:t_avg"])
    #                             cmd.append(str(tt) +
    #                                        ':' + str(self.compost_fan_config.relais_consigne_temperature_fan) +
    #                                        ":" + str(self.compost_fan_config.relais_etat) +
    #                                        ":" + str(self.relais_t_avg))
    #
    #                             subprocess.call(cmd)
    #                             self.graph_all_nodes()
                            elif b_array_header[2] == SEND_ALL_CFG:
                                cur_groupe_sonde = self.get_groupe_sonde_id(self.node_id)
                                print('lecture des configuration pour le groupe sonde : ' + str(cur_groupe_sonde))
                                print ('Node : ' + str(self.node_id))

                                # Lecture de PC1
                                b_bytes = self.ser.read(4)
                                temp_data = self.read_float(b_bytes)
                                self.lgs[cur_groupe_sonde].nfc.PC1 = temp_data
                                print ('PC1 : ' + str(self.lgs[cur_groupe_sonde].nfc.PC1))

                                # Lecture de PC2
                                b_bytes = self.ser.read(4)
                                temp_data = self.read_float(b_bytes)
                                self.lgs[cur_groupe_sonde].nfc.PC2 = temp_data
                                print ('PC2 : ' + str(self.lgs[cur_groupe_sonde].nfc.PC2))

                                # Lecture de PC3
                                b_bytes = self.ser.read(4)
                                temp_data = self.read_float(b_bytes)
                                self.lgs[cur_groupe_sonde].nfc.PC3 = temp_data
                                print ('PC3 : ' + str(self.lgs[cur_groupe_sonde].nfc.PC3))

                                # Lecture de PC4
                                b_bytes = self.ser.read(4)
                                temp_data = self.read_float(b_bytes)
                                self.lgs[cur_groupe_sonde].nfc.PC4 = temp_data
                                print ('PC4 : ' + str(self.lgs[cur_groupe_sonde].nfc.PC4))

                                # Lecture de TV1
                                b_bytes = self.ser.read(2)
                                temp_data = self.read_uint16(b_bytes)
                                self.lgs[cur_groupe_sonde].nfc.TV1 = temp_data
                                print ('TV1 : ' + str(self.lgs[cur_groupe_sonde].nfc.TV1))

                                # Lecture de TV2
                                b_bytes = self.ser.read(2)
                                temp_data = self.read_uint16(b_bytes)
                                self.lgs[cur_groupe_sonde].nfc.TV2 = temp_data
                                print ('TV2 : ' + str(self.lgs[cur_groupe_sonde].nfc.TV2))

                                # Lecture de TV3
                                b_bytes = self.ser.read(2)
                                temp_data = self.read_uint16(b_bytes)
                                self.lgs[cur_groupe_sonde].nfc.TV3 = temp_data
                                print ('TV3 : ' + str(self.lgs[cur_groupe_sonde].nfc.TV2))

                                # Lecture de TV4
                                b_bytes = self.ser.read(2)
                                temp_data = self.read_uint16(b_bytes)
                                self.lgs[cur_groupe_sonde].nfc.TV4 = temp_data
                                print ('TV4 : ' + str(self.lgs[cur_groupe_sonde].nfc.TV1))

                                # Lecture de TA1
                                b_bytes = self.ser.read(2)
                                temp_data = self.read_uint16(b_bytes)
                                self.lgs[cur_groupe_sonde].nfc.TA1 = temp_data
                                print ('TA1 : ' + str(self.lgs[cur_groupe_sonde].nfc.TA1))

                                # Lecture de TA2
                                b_bytes = self.ser.read(2)
                                temp_data = self.read_uint16(b_bytes)
                                self.lgs[cur_groupe_sonde].nfc.TA2 = temp_data
                                print ('TA2 : ' + str(self.lgs[cur_groupe_sonde].nfc.TA2))

                                # Lecture de TA3
                                b_bytes = self.ser.read(2)
                                temp_data = self.read_uint16(b_bytes)
                                self.lgs[cur_groupe_sonde].nfc.TA3 = temp_data
                                print ('TA3 : ' + str(self.lgs[cur_groupe_sonde].nfc.TA3))

                                # Lecture de TA4
                                b_bytes = self.ser.read(2)
                                temp_data = self.read_uint16(b_bytes)
                                self.lgs[cur_groupe_sonde].nfc.TA4 = temp_data
                                print ('TA4 : ' + str(self.lgs[cur_groupe_sonde].nfc.TA4))

                                # Lecture de delais_minute
                                b_bytes = self.ser.read(1)
                                temp_data = self.read_uint8(b_bytes)
                                self.lgs[cur_groupe_sonde].nfc.delais_minute = temp_data
                                print ('delais_minute: ' + str(self.lgs[cur_groupe_sonde].nfc.delais_minute))

                                # Lecture de node_compost_address_0
                                b_bytes = self.ser.read(1)
                                temp_data = self.read_uint8(b_bytes)
                                self.lgs[cur_groupe_sonde].nfc.node_compost_address_0 = temp_data
                                print ('node_compost_address_0: ' + str(self.lgs[cur_groupe_sonde].nfc.node_compost_address_0))

                                # Lecture de node_compost_address_1
                                b_bytes = self.ser.read(1)
                                temp_data = self.read_uint8(b_bytes)
                                self.lgs[cur_groupe_sonde].nfc.node_compost_address_1 = temp_data
                                print ('node_compost_address_1: ' + str(self.lgs[cur_groupe_sonde].nfc.node_compost_address_1))

                                # Lecture de node_compost_address_2
                                b_bytes = self.ser.read(1)
                                temp_data = self.read_uint8(b_bytes)
                                self.lgs[cur_groupe_sonde].nfc.node_compost_address_2 = temp_data
                                print ('node_compost_address_2: ' + str(self.lgs[cur_groupe_sonde].nfc.node_compost_address_2))

                                # Lecture de node_compost_address_3
                                b_bytes = self.ser.read(1)
                                temp_data = self.read_uint8(b_bytes)
                                self.lgs[cur_groupe_sonde].nfc.node_compost_address_3 = temp_data
                                print ('node_compost_address_3: ' + str(self.lgs[cur_groupe_sonde].nfc.node_compost_address_3))

                                # Lecture de node_compost_cfg_0
                                b_bytes = self.ser.read(1)
                                temp_data = self.read_uint8(b_bytes)
                                self.lgs[cur_groupe_sonde].nfc.node_compost_cfg_0 = temp_data
                                print ('node_compost_cfg_0: ' + str(self.lgs[cur_groupe_sonde].nfc.node_compost_cfg_0))

                                # Lecture de node_compost_cfg_1
                                b_bytes = self.ser.read(1)
                                temp_data = self.read_uint8(b_bytes)
                                self.lgs[cur_groupe_sonde].nfc.node_compost_cfg_1 = temp_data
                                print ('node_compost_cfg_1: ' + str(self.lgs[cur_groupe_sonde].nfc.node_compost_cfg_1))

                                # Lecture de node_compost_cfg_2
                                b_bytes = self.ser.read(1)
                                temp_data = self.read_uint8(b_bytes)
                                self.lgs[cur_groupe_sonde].nfc.node_compost_cfg_2 = temp_data
                                print ('node_compost_cfg_2: ' + str(self.lgs[cur_groupe_sonde].nfc.node_compost_cfg_2))

                                # Lecture de node_compost_cfg_3
                                b_bytes = self.ser.read(1)
                                temp_data = self.read_uint8(b_bytes)
                                self.lgs[cur_groupe_sonde].nfc.node_compost_cfg_3 = temp_data
                                print ('node_compost_cfg_3: ' + str(self.lgs[cur_groupe_sonde].nfc.node_compost_cfg_3))

                                text_array = bytearray(16)
                                for i in range(16):
                                    b_bytes = self.ser.read(1)
                                    text_array[i] = b_bytes
                                self.lgs[cur_groupe_sonde].nfc.node_compost_txt_0 = text_array.decode("utf-8")
                                text_array = bytearray(16)
                                for i in range(16):
                                    b_bytes = self.ser.read(1)
                                    text_array[i] = b_bytes
                                self.lgs[cur_groupe_sonde].nfc.node_compost_txt_1 = text_array.decode("utf-8")
                                text_array = bytearray(16)
                                for i in range(16):
                                    b_bytes = self.ser.read(1)
                                    text_array[i] = b_bytes
                                self.lgs[cur_groupe_sonde].nfc.node_compost_txt_2 = text_array.decode("utf-8")
                                text_array = bytearray(16)
                                for i in range(16):
                                    b_bytes = self.ser.read(1)
                                    text_array[i] = b_bytes
                                self.lgs[cur_groupe_sonde].nfc.node_compost_txt_3 = text_array.decode("utf-8")

    #                            nb_bytes = self.ser.inWaiting()
    #                            b_bytes = self.ser.read(nb_bytes)
    #                            print("Nombre de byte restant... : " + str(nb_bytes))
    #                            time.sleep(1)
                                b_array_set_message_data = bytearray(5)
                                b_array_set_message_data[0] = FEATHER_MSG_HEADER
                                b_array_set_message_data[1] = FEATHER_MSG_SEND_ALL_TEMP
                                b_array_set_message_data[2] = self.node_id
                                b_array_set_message_data[3] = SEND_ALL_TEMP
                                b_array_set_message_data[4] = FEATHER_MSG_END
                                print('Envoie la commande SEND_ALL_TEMP')
                                self.writeserialdata(b_array_set_message_data)

                            elif b_array_header[2] == FEATHER_MSG_SEND_COMPOST_NODE_DATA:
                                cur_groupe_sonde = self.get_groupe_sonde_id(self.node_id)
                                print('lecture des donnes pour le groupe sonde : ' + str(cur_groupe_sonde))

                                # Lecture de compost_node_id
                                b_bytes = self.ser.read(1)
                                compost_node_id = self.read_uint8(b_bytes)

                                # Lecture de compost_node_address
                                b_bytes = self.ser.read(1)
                                temp_data = self.read_uint8(b_bytes)
                                self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].node_address = temp_data
                                print ('Node ' + str(compost_node_id) + ' : node_address : '
                                       + str(self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].node_address))

                                # Lecture de node_cfg
                                b_bytes = self.ser.read(1)
                                temp_data = self.read_uint8(b_bytes)
                                self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].node_cfg = temp_data
                                print ('Node ' + str(compost_node_id) + ' : node_cfg : '
                                       + str(self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].node_cfg))

                                # Lecture du timestamp
                                b_bytes = self.ser.read(4)
                                temp_data = self.read_uint32(b_bytes)
                                self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].timestamp = temp_data
                                print ('Node ' + str(compost_node_id) + ' : timestamp : '
                                       + str(self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].timestamp))

                                # Lecture de ntc_1
                                b_bytes = self.ser.read(4)
                                temp_data = self.read_float(b_bytes)
                                self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].ntc_1 = temp_data
                                print ('Node ' + str(compost_node_id) + ' : ntc_1 : '
                                       + str(self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].ntc_1))

                                # Lecture de ntc_2
                                b_bytes = self.ser.read(4)
                                temp_data = self.read_float(b_bytes)
                                self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].ntc_2 = temp_data
                                print ('Node ' + str(compost_node_id) + ' : ntc_2 : '
                                       + str(self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].ntc_2))

                                # Lecture de bme_humidity
                                b_bytes = self.ser.read(4)
                                temp_data = self.read_float(b_bytes)
                                self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].bme_humidity = temp_data
                                print ('Node ' + str(compost_node_id) + ' : bme_humidity : '
                                       + str(self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].bme_humidity))

                                # Lecture de bme_temp
                                b_bytes = self.ser.read(4)
                                temp_data = self.read_float(b_bytes)
                                self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].bme_temp = temp_data
                                print ('Node ' + str(compost_node_id) + ' : bme_temp : '
                                       + str(self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].bme_temp))

                                # Lecture de bme_pression
                                b_bytes = self.ser.read(4)
                                temp_data = self.read_float(b_bytes)
                                self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].bme_pression = temp_data
                                print ('Node ' + str(compost_node_id) + ' : bme_pression : '
                                       + str(self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].bme_pression))

                                # Lecture de conductivite
                                b_bytes = self.ser.read(2)
                                temp_data = self.read_uint16(b_bytes)
                                self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].conductivite = temp_data
                                print ('Node ' + str(compost_node_id) + ' : conductivite : '
                                       + str(self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].conductivite))

                                # Lecture de batt_voltage
                                b_bytes = self.ser.read(4)
                                temp_data = self.read_float(b_bytes)
                                self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].batt_voltage = temp_data
                                print ('Node ' + str(compost_node_id) + ' : batt_voltage : '
                                       + str(self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].batt_voltage))

                                # Lecture de delay_minutes
                                b_bytes = self.ser.read(1)
                                temp_data = self.read_uint8(b_bytes)
                                self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].delay_minutes = temp_data
                                print ('Node ' + str(compost_node_id) + ' : delay_minutes : '
                                       + str(self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].delay_minutes))

                                # Lecture de txpower
                                b_bytes = self.ser.read(1)
                                temp_data = self.read_uint8(b_bytes)
                                self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].txpower = temp_data
                                print ('Node ' + str(compost_node_id) + ' : txpower : '
                                       + str(self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].txpower))

                                # Lecture de last_rssi
                                b_bytes = self.ser.read(1)
                                temp_data = self.read_int8(b_bytes)
                                self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].last_rssi = temp_data
                                print ('Node ' + str(compost_node_id) + ' : last_rssi : '
                                       + str(self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].last_rssi))

                                # Lecture de new_data
                                b_bytes = self.ser.read(1)
                                temp_data = self.read_uint8(b_bytes)
                                self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].new_data = temp_data
                                print ('Node ' + str(compost_node_id) + ' : new_data : '
                                       + str(self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].new_data))

                                # Lecture de clock_ok
                                b_bytes = self.ser.read(1)
                                temp_data = self.read_uint8(b_bytes)
                                self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].clock_ok = temp_data
                                print ('Node ' + str(compost_node_id) + ' : clock_ok : '
                                       + str(self.lgs[cur_groupe_sonde].list_compost_node_data[compost_node_id].clock_ok))


                            elif b_array_header[2] == TEMP_1:
                                print ('READ_TEMP_1')
                                b_bytes = self.ser.read(4)   # Lecture de 4 bytes pour temperature
                                line = line + "  " + ':'.join('{:02x}'.format(ord(c)) for c in b_bytes)
                                print line
                                b_array_data = bytearray(b_bytes)
                                end_msg = self.ser.read(1)
                                b = ''.join(chr(i) for i in b_array_data)
                                import struct
                                t1_float = struct.unpack('>f', b)
                                f_value = t1_float[0]
                                print f_value
                                with open(compost_csv_file, 'ab') as csvfile:
                                    compost_data_writer = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
                                    compost_data_writer.writerow([timestamp, b_array_header[0], 't_1', f_value])
                                if self.node_id == 1:
                                    self.compost_fan_data.node_1_temperature_surface = f_value
                                elif self.node_id == 2:
                                    self.compost_fan_data.node_2_temperature_surface = f_value
                                elif self.node_id == 3:
                                    self.compost_fan_data.node_3_temperature_surface = f_value
                                elif self.node_id == 4:
                                    self.compost_fan_data.node_4_temperature_surface = f_value
                            elif b_array_header[2] == TEMP_2:
                                print ('READ_TEMP_2')
                                b_bytes = self.ser.read(4)   # Lecture de 4 bytes pour temperature
                                line = line + "  " + ':'.join('{:02x}'.format(ord(c)) for c in b_bytes)
                                print line
                                b_array_data = bytearray(b_bytes)
                                end_msg = self.ser.read(2)
                                b = ''.join(chr(i) for i in b_array_data)
                                import struct
                                t2_float = struct.unpack('>f', b)
                                f_value = (t2_float)[0]
                                print f_value
                                with open(compost_csv_file, 'ab') as csvfile:
                                    compost_data_writer = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
                                    compost_data_writer.writerow([timestamp, b_array_header[0], 't_2', f_value])
                                if self.node_id == 1:
                                    self.compost_fan_data.node_1_temperature_profondeur = f_value
                                elif self.node_id == 2:
                                    self.compost_fan_data.node_2_temperature_profondeur = f_value
                                elif self.node_id == 3:
                                    self.compost_fan_data.node_3_temperature_profondeur = f_value
                                elif self.node_id == 4:
                                    self.compost_fan_data.node_4_temperature_profondeur = f_value
                            elif b_array_header[2] == TEMP_3:
                                print ('READ_TEMP_3')
                                b_bytes = self.ser.read(4)   # Lecture de 4 bytes pour temperature
                                line = line + "  " + ':'.join('{:02x}'.format(ord(c)) for c in b_bytes)
                                print line
                                b_array_data = bytearray(b_bytes)
                                end_msg = self.ser.read(1)
                                b = ''.join(chr(i) for i in b_array_data)
                                import struct
                                t3_float = struct.unpack('>f', b)
                                f_value = (t3_float)[0]
                                print f_value
                                with open(compost_csv_file, 'ab') as csvfile:
                                    compost_data_writer = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
                                    compost_data_writer.writerow([timestamp, b_array_header[0], 't_3', f_value])
                                if self.node_id == 1:
                                    self.compost_fan_data.node_1_temperature_ambiant = f_value
                            elif b_array_header[2] == HUMIDITY_1:
                                print ('READ_HUMIDITY_1')
                                b_bytes = self.ser.read(4)   # Lecture de 4 bytes pour temperature
                                line = line + "  " + ':'.join('{:02x}'.format(ord(c)) for c in b_bytes)
                                print line
                                b_array_data = bytearray(b_bytes)
                                end_msg = self.ser.read(2)
                                b = ''.join(chr(i) for i in b_array_data)
                                import struct
                                h1_float = struct.unpack('>f', b)
                                f_value = (h1_float)[0]
                                print f_value
                                with open(compost_csv_file, 'ab') as csvfile:
                                    compost_data_writer = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
                                    compost_data_writer.writerow([timestamp, b_array_header[0], 'h_1', f_value])
                                if self.node_id == 1:
                                    self.compost_fan_data.node_1_humidity_ambiant = f_value
                            elif b_array_header[2] == READ_BATTERY_VOLTAGE:
                                print ('READ_BATTERY_VOLTAGE')
                                b_bytes = self.ser.read(4)   # Lecture de 4 bytes pour temperature
                                line = line + "  " + ':'.join('{:02x}'.format(ord(c)) for c in b_bytes)
                                print line
                                b_array_data = bytearray(b_bytes)
                                end_msg = self.ser.read(2)
                                b = ''.join(chr(i) for i in b_array_data)
                                import struct
                                bat_float = struct.unpack('>f', b)

                                f_value = (bat_float)[0]
                                print f_value
                                with open(compost_csv_file, 'ab') as csvfile:
                                    compost_data_writer = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
                                    compost_data_writer.writerow([timestamp, b_array_header[0], 'bat', f_value])
                                if self.node_id == 1:
                                    self.compost_fan_data.node_1_battery_voltage = f_value
                                elif self.node_id == 2:
                                    self.compost_fan_data.node_2_battery_voltage = f_value
                                elif self.node_id == 3:
                                    self.compost_fan_data.node_3_battery_voltage = f_value
                                elif self.node_id == 4:
                                    self.compost_fan_data.node_4_battery_voltage = f_value
                            else:
                                print('No Msg Data')
                                b_bytes = self.ser.read(6)   # Lecture de 4 bytes pour temperature
                                line = line + "  " + ':'.join('{:02x}'.format(ord(c)) for c in b_bytes)
                                print line
                        elif b_array_header[0] == FEATHER_MSG_QUERY_DATA:
                            print ('***** Receive query data *****')
                            if b_array_header[2] == TEMP_1:
                                print ('READ_TEMP_1')
                            elif b_array_header[2] == TEMP_2:
                                print ('READ_TEMP_2')
                            elif b_array_header[2] == TEMP_3:
                                print ('READ_TEMP_3')
                            else:
                                print ('Query data : ' + str(b_array_header[2]))
                            b_bytes = self.ser.read(1)
                        elif b_array_header[0] == FEATHER_MSG_GET_DATA:
                            print ('***** Receive get data *****')
                        elif b_array_header[0] == FEATHER_MSG_SET_DATA:
                            print ('***** Receive set data *****')
                    else:
                        print ("Serial receive : " + str(self.read_uint8(b_bytes)))
                        nb_bytes = self.ser.inWaiting()
                        flush_byte = self.ser.read(nb_bytes)
                        print ("flush byte : " + str(nb_bytes))

    #                    print
    #                    print
                time.sleep(0.5)
        except KeyboardInterrupt:
            GPIO.cleanup()
           #     self.d_tt = int(time.time()) - tt

    def dispatcher(self):
        print ('dispatcher')
        thread.start_new_thread(self.readserialdata, ())
        while True:
            print('Waiting for client connection...')
            connection, address = self.sock.accept()
            print ('Server connected by ', address)
            print ('at', self.now())
            thread.start_new_thread(self.handleClient, (connection,))

    def graph_all_nodes(self):
        print("Graph All Nodes")
        #        info = rrdtool.info("node_00.rrd")
        #        print info
        size_w = 640
        size_h = 480
        start_time = "end-12h"
        end_time = "now"

        for x in range(4):
            print "graph node : " + '{:02d}'.format(x)
            rrdtool.graph("node_" + '{:02d}'.format(x) + ".png",
                          "--imgformat", "PNG",
                          "--alt-autoscale",
                          "--title", "Node " + '{:02d}'.format(x) + "",
                          "-w", str(size_w),
                          "-h", str(size_h),
                          "-s", start_time,
                          "-e", end_time,
                          "--vertical-label", "Temperature (degre C)",
                          "--right-axis-label", "Temperature (degre C)",
                          "--right-axis-format", "%.1lf",
                          "--right-axis", "1:0",
                          "--no-gridfit",
                          "DEF:node_" + '{:02d}'.format(x) + "_t_1=node_" + '{:02d}'.format(x) + ".rrd:t_1:AVERAGE",
                          "DEF:node_" + '{:02d}'.format(x) + "_t_2=node_" + '{:02d}'.format(x) + ".rrd:t_2:AVERAGE",
                          "DEF:node_" + '{:02d}'.format(x) + "_t_3=node_" + '{:02d}'.format(x) + ".rrd:t_3:AVERAGE",
                          "DEF:node_" + '{:02d}'.format(x) + "_h_1=node_" + '{:02d}'.format(x) + ".rrd:h_1:AVERAGE",
                          "DEF:node_" + '{:02d}'.format(x) + "_conductivite=node_" + '{:02d}'.format(x) + ".rrd:conductivite:AVERAGE",
                          "DEF:node_" + '{:02d}'.format(x) + "_txpower=node_" + '{:02d}'.format(x) + ".rrd:txpower:AVERAGE",
                          "DEF:node_" + '{:02d}'.format(x) + "_last_rssi=node_" + '{:02d}'.format(x) + ".rrd:last_rssi:AVERAGE",
                          "DEF:node_" + '{:02d}'.format(x) + "_batt_voltage=node_" + '{:02d}'.format(x) + ".rrd:batt_voltage:AVERAGE",
                          "LINE1:node_" + '{:02d}'.format(x) + "_t_1#FC0303:Node " + '{:02d}'.format(x) + " - Temperature de surface",
                          "LINE1:node_" + '{:02d}'.format(x) + "_t_2#FC5A03:Node " + '{:02d}'.format(x) + " - Temperature de profondeur",
                          "LINE1:node_" + '{:02d}'.format(x) + "_t_3#FC9E03:Node " + '{:02d}'.format(x) + " - Temperature du boitier",
                          "LINE1:node_" + '{:02d}'.format(x) + "_h_1#FC4F03:Node " + '{:02d}'.format(x) + " - humidite du boitier",
                          "LINE1:node_" + '{:02d}'.format(x) + "_conductivite#114F03:Node " + '{:04d}'.format(x) + " - Conductivite",
                          "LINE1:node_" + '{:02d}'.format(x) + "_txpower#FC2F03:Node " + '{:02d}'.format(x) + " - txpower",
                          "LINE1:node_" + '{:02d}'.format(x) + "_last_rssi#FC1F03:Node " + '{:02d}'.format(x) + " - last_rssi",
                          "LINE1:node_" + '{:02d}'.format(x) + "_batt_voltage#FCA903:Node " + '{:02d}'.format(x) + " - Battery voltage")

    def graph_node_1(self):
        print("Graph node 1")
#        info = rrdtool.info("node_00.rrd")
#        print info
        size_w = 640
        size_h = 480
        start_time = "end-12h"
        end_time = "now"

        rrdtool.graph("node_1.png",
                      "--imgformat", "PNG",
                      "--alt-autoscale",
                      "--title", "Node 0",
                      "-w", str(size_w),
                      "-h", str(size_h),
                      "-s", start_time,
                      "-e", end_time,
                      "--vertical-label", "Temperature (degre C)",
                      "--right-axis-label", "Temperature (degre C)",
                      "--right-axis-format", "%.1lf",
                      "--right-axis", "1:0",
                      "--no-gridfit",
                      "DEF:node_00_t_1=node_00.rrd:t_1:AVERAGE",
                      "DEF:node_00_t_2=node_00.rrd:t_2:AVERAGE",
                      "DEF:node_00_t_3=node_00.rrd:t_3:AVERAGE",
                      "DEF:node_00_h_1=node_00.rrd:h_1:AVERAGE",
                      "DEF:node_00_pression=node_00.rrd:pression:AVERAGE",
                      "DEF:node_00_conductivite=node_00.rrd:conductivite:AVERAGE",
                      "DEF:node_00_last_rssi=node_00.rrd:last_rssi:AVERAGE",
                      "DEF:node_00_last_rssi=node_00.rrd:last_rssi:AVERAGE",
                      "DEF:node_00_batt_voltage=node_00.rrd:batt_voltage:AVERAGE",
                      "LINE1:node_00_t_surface#FC0303:Node 00 - Temperature de surface",
                      "LINE1:node_00_t_profondeur#FC5A03:Node 00 - Temperature de profondeur",
                      "LINE1:node_00_t_air#FC9E03:Node 00 - Temperature air",
                      "LINE1:node_00_humidity#FC4F03:Node 00 - Humidite air",
                      "LINE1:node_00_batt_voltage#FCA903:Node 00 - Battery voltage")

    def graph_node_2(self):
        print("Graph node 1")
        #        info = rrdtool.info("node_00.rrd")
        #        print info
        size_w = 640
        size_h = 480
        start_time = "end-12h"
        end_time = "now"
        rrdtool.graph("node_1.png",
                      "--imgformat", "PNG",
                      "--alt-autoscale",
                      "--title", "Node 1",
                      "-w", str(size_w),
                      "-h", str(size_h),
                      "-s", start_time,
                      "-e", end_time,
                      "--vertical-label", "Temperature (degre C)",
                      "--right-axis-label", "Temperature (degre C)",
                      "--right-axis-format", "%.1lf",
                      "--right-axis", "1:0",
                      "--no-gridfit",
                      "DEF:node_01_t_surface=node_01.rrd:t_surface:AVERAGE",
                      "DEF:node_01_t_profondeur=node_01.rrd:t_profondeur:AVERAGE",
                      "DEF:node_01_batt_voltage=node_01.rrd:batt_voltage:AVERAGE",
                      "LINE1:node_01_t_surface#FC0303:Node 01 - Temperature de surface",
                      "LINE1:node_01_t_profondeur#FC5A03:Node 01 - Temperature de profondeur",
                      "LINE1:node_01_batt_voltage#FCA903:Node 01 - Battery voltage")

    def set_relais_cfg(self, groupe_sonde):
        self.lgs[groupe_sonde].nfc = pickle.loads(self.socket_data)
        node_fan_address = self.lgs[groupe_sonde].node_fan_address
        print("set_relais_cfg() : node_fan_address : " + str(node_fan_address))
        b_array_set_message_data = bytearray(111)
        b_array_set_message_data[0] = FEATHER_MSG_HEADER
        b_array_set_message_data[1] = FEATHER_MSG_SET_DATA
        b_array_set_message_data[2] = node_fan_address
        b_array_set_message_data[3] = RELAIS_CFG
        ba = bytearray(struct.pack('f', self.lgs[groupe_sonde].nfc.PC1))
        b_array_set_message_data[4] = ba[3]
        b_array_set_message_data[5] = ba[2]
        b_array_set_message_data[6] = ba[1]
        b_array_set_message_data[7] = ba[0]
        ba = bytearray(struct.pack('f', self.lgs[groupe_sonde].nfc.PC2))
        b_array_set_message_data[8] = ba[3]
        b_array_set_message_data[9] = ba[2]
        b_array_set_message_data[10] = ba[1]
        b_array_set_message_data[11] = ba[0]
        ba = bytearray(struct.pack('f', self.lgs[groupe_sonde].nfc.PC3))
        b_array_set_message_data[12] = ba[3]
        b_array_set_message_data[13] = ba[2]
        b_array_set_message_data[14] = ba[1]
        b_array_set_message_data[15] = ba[0]
        ba = bytearray(struct.pack('f', self.lgs[groupe_sonde].nfc.PC4))
        b_array_set_message_data[16] = ba[3]
        b_array_set_message_data[17] = ba[2]
        b_array_set_message_data[18] = ba[1]
        b_array_set_message_data[19] = ba[0]
        ba = bytearray(struct.pack('H', self.lgs[groupe_sonde].nfc.TV1))
        b_array_set_message_data[20] = ba[1]
        b_array_set_message_data[21] = ba[0]
        ba = bytearray(struct.pack('H', self.lgs[groupe_sonde].nfc.TV2))
        b_array_set_message_data[22] = ba[1]
        b_array_set_message_data[23] = ba[0]
        ba = bytearray(struct.pack('H', self.lgs[groupe_sonde].nfc.TV3))
        b_array_set_message_data[24] = ba[1]
        b_array_set_message_data[25] = ba[0]
        ba = bytearray(struct.pack('H', self.lgs[groupe_sonde].nfc.TV4))
        b_array_set_message_data[26] = ba[1]
        b_array_set_message_data[27] = ba[0]
        ba = bytearray(struct.pack('H', self.lgs[groupe_sonde].nfc.TA1))
        b_array_set_message_data[28] = ba[1]
        b_array_set_message_data[29] = ba[0]
        ba = bytearray(struct.pack('H', self.lgs[groupe_sonde].nfc.TA2))
        b_array_set_message_data[30] = ba[1]
        b_array_set_message_data[31] = ba[0]
        ba = bytearray(struct.pack('H', self.lgs[groupe_sonde].nfc.TA3))
        b_array_set_message_data[32] = ba[1]
        b_array_set_message_data[33] = ba[0]
        ba = bytearray(struct.pack('H', self.lgs[groupe_sonde].nfc.TA4))
        b_array_set_message_data[34] = ba[1]
        b_array_set_message_data[35] = ba[0]
        ba = bytearray(struct.pack('B', self.lgs[groupe_sonde].nfc.delais_minute))
        b_array_set_message_data[36] = ba[0]
        ba = bytearray(struct.pack('B', self.lgs[groupe_sonde].nfc.node_compost_address_0))
        b_array_set_message_data[37] = ba[0]
        ba = bytearray(struct.pack('B', self.lgs[groupe_sonde].nfc.node_compost_address_1))
        b_array_set_message_data[38] = ba[0]
        ba = bytearray(struct.pack('B', self.lgs[groupe_sonde].nfc.node_compost_address_2))
        b_array_set_message_data[39] = ba[0]
        ba = bytearray(struct.pack('B', self.lgs[groupe_sonde].nfc.node_compost_address_3))
        b_array_set_message_data[40] = ba[0]
        ba = bytearray(struct.pack('B', self.lgs[groupe_sonde].nfc.node_compost_cfg_0))
        b_array_set_message_data[41] = ba[0]
        ba = bytearray(struct.pack('B', self.lgs[groupe_sonde].nfc.node_compost_cfg_1))
        b_array_set_message_data[42] = ba[0]
        ba = bytearray(struct.pack('B', self.lgs[groupe_sonde].nfc.node_compost_cfg_2))
        b_array_set_message_data[43] = ba[0]
        ba = bytearray(struct.pack('B', self.lgs[groupe_sonde].nfc.node_compost_cfg_3))
        b_array_set_message_data[44] = ba[0]

        jj = 45
        text_array = bytearray(16)
        ta = bytearray(self.lgs[groupe_sonde].nfc.node_compost_txt_0, 'utf8')
        ta_len = len(ta)
        for y in range(ta_len):
            text_array[y] = ta[y]
        for x in range(16):
            b_array_set_message_data[jj+x] = text_array[x]

        jj = jj + 16
        ta = bytearray(self.lgs[groupe_sonde].nfc.node_compost_txt_1, 'utf8')
        ta_len = len(ta)
        for y in range(ta_len):
            text_array[y] = ta[y]
        for x in range(16):
            b_array_set_message_data[jj+x] = text_array[x]

        jj = jj + 16
        ta = bytearray(self.lgs[groupe_sonde].nfc.node_compost_txt_2, 'utf8')
        ta_len = len(ta)
        for y in range(ta_len):
            text_array[y] = ta[y]
        for x in range(16):
            b_array_set_message_data[jj+x] = text_array[x]

        jj = jj + 16
        ta = bytearray(self.lgs[groupe_sonde].nfc.node_compost_txt_3, 'utf8')
        ta_len = len(ta)
        for y in range(ta_len):
            text_array[y] = ta[y]
        for x in range(16):
            b_array_set_message_data[jj+x] = text_array[x]

        b_array_set_message_data[jj+16] = FEATHER_MSG_END
        print('Envoie de la nouvelle configuration...')
        self.writeserialdata(b_array_set_message_data)

    def set_relais_date_time(self, node_id):
        utc = int(time.time())
#        local = utc - (60*60*5)
        ba = bytearray(struct.pack('i', utc))
        b_array_set_message_data = bytearray(8)
        b_array_set_message_data[0] = FEATHER_MSG_HEADER
        b_array_set_message_data[1] = FEATHER_MSG_SET_CLOCK
        b_array_set_message_data[2] = node_id
        b_array_set_message_data[3] = ba[3]
        b_array_set_message_data[4] = ba[2]
        b_array_set_message_data[5] = ba[1]
        b_array_set_message_data[6] = ba[0]
        b_array_set_message_data[7] = FEATHER_MSG_END
        print('Envoie la datetime au noeud : ' + str(node_id))
        self.writeserialdata(b_array_set_message_data)

    def get_groupe_sonde_id(self, node_fan_address):
        #print (node_fan_address)
        current_ssr_found = False
        current_groupe_sonde = 0
        for x in range(0, 4):
            if self.lgs[x].node_fan_address == node_fan_address:
                current_ssr_found = True
                current_groupe_sonde = x
                print ('node_fan  : ' + str(node_fan_address) + 'trouve')
        if current_ssr_found:
            return current_groupe_sonde


#  Main

if __name__ == '__main__':
    try:
        compostFAN = CompostFAN(1)
        compostFAN.init_serial()
    #    compostFAN.set_relais_date_time()
        compostFAN.init_server()
        compostFAN.dispatcher()
    except KeyboardInterrupt:
        GPIO.cleanup()


#    compostFAN.readserialdata()
#    compostFAN.read_test()
