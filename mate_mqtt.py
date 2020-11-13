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
import paho.mqtt.client as mqtt
# import paho.mqtt.publish as publish
import schedule
from datetime import datetime
import pytz
from suntime import Sun




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
        self.ser.flushInput()  # make sure it's fresh data
        while (not (fxfound and mxfound)):
            ln = self.ser.readline().strip()
            if ln.startswith('C'):
                try:
                    mx = MXStatusPacket(ln)
                    mxfound = True
                except IndexError as e:
                    print(e)
            if ln.startswith('1'):
                try:
                    fx = FXStatusPacket(ln)
                    fxfound = True
                except IndexError as e:
                    print(e)
        return mx, fx

    def read_raw(self):
        return self.ser.readline().strip()


broker = '127.0.0.1'
state_topic = 'home-assistant/battery/voltage'
delay = 60.0

client = mqtt.Client("ha-client")
client.connect(broker)
client.loop_start()

mate = MateCom('/dev/ttyUSB0')

starttime = time.time()

latitude = 47.8408
longitude = -120.0168

sun = Sun(latitude, longitude)

# Get today's sunrise and sunset in UTC
today_sr = sun.get_sunrise_time()
today_ss = sun.get_sunset_time()

#
#  Since our sunrise is in reference to UTC the hour can be before or after
#  and we can get off by a day so we will only use time portion, not the date 
#
def sunup():
    sr = today_sr.time()
    ss = today_ss.time()
    now = datetime.now(pytz.utc).time()
    sunup = False
    if (ss < sr):
        if (sr < now) or (now < ss):
            sunup = True
    else:
        if (sr < now < ss):
            sunup = True
    return sunup


def sundown():
    sr = today_sr.time()
    ss = today_ss.time()
    now = datetime.now(pytz.utc).time()
    sundown = False
    if (sr < ss):
        if (ss < now) or (now < sr):
            sundown = True
    else:
        if (ss < now < sr):
            sundown = True
    return sundown

def midnight():
    pass


sunstatus = False
sunupkhw = 0.0
sundownkhw = 0.0


def job():
    global client, sunstatus, sunupkhw, sundownkhw
    # print("checking sunrise/sundown")
    # print(today_ss.time())
    # print(today_sr.time())
    # print(datetime.now(timezone.utc).time())
    mx, fx = mate.read_all()
    client.publish("home-assistant/mx/charge/current",
                   float(mx.charge_current))
    client.publish("home-assistant/mx/pv/current", float(mx.pv_current))
    client.publish("home-assistant/mx/pv/voltage", float(mx.pv_voltage))
    client.publish("home-assistant/fx/invertercurrent/current",
                   float(fx.inverter_current))
    client.publish("home-assistant/fx/charger/current",
                   float(fx.charger_current))
    client.publish("home-assistant/fx/ac/input/voltage",
                   float(fx.ac_input_voltage))
    client.publish("home-assistant/fx/ac/output/voltage",
                   float(fx.ac_output_voltage))
    client.publish("home-assistant/fx/batt/voltage", float(fx.batt_voltage))
    if sunup():
        if not sunstatus:
            print(datetime.now(pytz.utc).time())
            print("The sun is up...")
            sunupkhw = 0.0
            sunstatus = True
        sunupkhw += (float(fx.inverter_current)+1.0)*float(fx.ac_output_voltage)/60.0
        client.publish("home-assistant/sunup/khw", sunupkhw)
    if sundown():
        if sunstatus:
            print(datetime.now(pytz.utc).time())
            print("The sun is down...")
            sundownkhw = 0.0
            sunstatus = False
        sundownkhw += (float(fx.inverter_current)+1.0)*float(fx.ac_output_voltage)/60.0
        client.publish("home-assistant/sundown/khw", sundownkhw)


schedule.every(1).minutes.do(job)
# schedule.every().hour.do(job)
#schedule.every().day.at("18:51").do(job)

broker = '127.0.0.1'
state_topic = 'home-assistant/battery/voltage'
delay = 60.0

client = mqtt.Client("ha-client")
client.connect(broker)
client.loop_start()

starttime = time.time()


while 1:
    schedule.run_pending()
    time.sleep(1)
    
