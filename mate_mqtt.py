# Forked from pyMate controller
# Author: Stephen Hicks
#
# Allows communication with an Outback Systems MATE controller and
# publshing values local using MQTT

# Currently only supports the FX and MX status page 
#
# NOTE: This is intended for communication with the MATE's RS232 port, not Outback's proprietary protocol.

import serial
from value import Value
import time
import random
import paho.mqtt.client as mqtt
# import paho.mqtt.publish as publish


class MXStatusPacket(object):
    """
    Represents an MX status packet, containing useful information
    such as charge current and PV voltage.
    """
    def __init__(self, packet):
        fields = packet.split(',')
        self.address = fields[0]
        # fields[1] unused
        self.charge_current = Value(float(fields[2]) + (float(fields[6]) / 10.0), 'A', resolution=1)
        self.pv_current = Value(fields[3], 'A', resolution=0)
        self.pv_voltage = Value(fields[4], 'V', resolution=0)
        self.daily_kwh = Value(float(fields[5]) / 10.0, 'kWh', resolution=1)
        self.aux_mode = fields[7]
        self.error_mode = fields[8]
        self.charger_mode = fields[9]
        self.bat_voltage = Value(float(fields[10]) / 10, 'V', resolution=1)
        self.daily_ah = Value(float(fields[11]), 'Ah', resolution=1)
        # fields[12] unused

        chk_expected = int(fields[13])
        chk_actual = sum(ord(x)-48 for x in packet[:-4] if ord(x) >= 48)
        if chk_expected != chk_actual:
            raise Exception("Checksum error in received packet")


class FXStatusPacket(object):
    """
    Represents an MX status packet, containing useful information
    such as charge current and PV voltage.
    """
    def __init__(self, packet):
        fields = packet.split(',')
        self.address = fields[0]
        # fields[1] unused
        self.inverter_current = Value(float(fields[1]), 'A', resolution=0)
        self.charger_current = Value(fields[2], 'A', resolution=0)
        self.ac_input_voltage = Value(fields[4], 'V', resolution=0)
        self.ac_output_voltage = Value(fields[5], 'V', resolution=0)
        self.batt_voltage = Value(float(fields[11]), 'V', resolution=0)
        # fields[12] unused

        chk_expected = int(fields[13])
        chk_actual = sum(ord(x)-48 for x in packet[:-4] if ord(x) >= 48)
        if chk_expected != chk_actual:
            raise Exception("Checksum error in received packet")


class MateCom(object):
    """
    Interfaces with the MATE controller on a specific COM port.
    Must be a proper RS232 port with RTS/DTR pins.
    """
    def __init__(self, port, baudrate=19200):
        self.ser = serial.Serial(port, baudrate, timeout=2)

        # Provide power to the Mate controller
        self.ser.setDTR(True)
        self.ser.setRTS(False)

        self.ser.readline()

    def read_status(self):
        ln = self.ser.readline().strip()
        if ln.startswith('C'):
            return "MX: ", MXStatusPacket(ln)
        if ln.startswith('1'):
            return "FX: ", FXStatusPacket(ln)
        else:
            return None, None

    def read_all(self):
        fx = None
        mx = None
        fxfound = False
        mxfound = False
        while (not (fxfound and mxfound)):
            ln = self.ser.readline().strip()
            if ln.startswith('C'):
                mx = MXStatusPacket(ln)
                mxfound = True
            if ln.startswith('1'):
                fx = FXStatusPacket(ln)
                fxfound = True
        return mx, fx

    def read_raw(self):
        return self.ser.readline().strip()


broker = '127.0.0.1'
state_topic = 'home-assistant/battery/voltage'
delay = 60

client = mqtt.Client("ha-client")
client.connect(broker)
client.loop_start()

mate = MateCom('/dev/ttyUSB0')

while True:
    mx, fx = mate.read_all()
    client.publish(state_topic, float(mx.bat_voltage))
    time.sleep(delay)
