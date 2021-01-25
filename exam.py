I2c

from smbus import SMBus
import sqlite3
import datetime

db = sqlite3.connect("external")
cur = db.cursor()
cur.execute("create table if not exists tbli2c(i2cID integer primary key autoincrement,message text,create_dt current_timestamp)")
db.commit()
addr = 0x8
bus = SMBus(1)

num = 1

print("Enter 1 for ON or 0 for OFF")
while num == 1:
    ledstate = input(">>>>>    ")
    if ledstate == "1":
        bus.write_byte(addr,0x1)
        block = bus.read_byte_data(8,1)
        print(block)
        sql = """insert into tbli2c(message,create_dt) values(?,?);"""
        data_sql = (block,datetime.datetime.now())
        cur.execute(sql,data_sql)
        db.commit()
    else ledstate == "0":
        bus.write(addr,0x0)
    else:import
        num = 0

Spi

import spidev
import time
import datetime
import sqlite3

db = sqlite3.connect("external")
cur = db.cursor()

spi = spidev.SpiDev(0,0)
spi.open(0,0)
msg = 0xAA

spi.max_speed_hz = 115200
while 1:
    spi.writebytes([0x4,0x86])
    data = spi.readbytes(1)
    print(data)
    sql = """insert into tblspi(msg,created_dt) values(?,?);"""
    data_sql = (data,datetime.datetime.now())
    cur.execute(sql,data_sql)
    db.commit()
    time.sleep(1)

publish

import context
import paho.mqtt.client as mqtt
import time

def on_connect(client,userdata,flag,rc):
    print("Connected with result code{rc}")
    
client = mqtt.Client()
client.on_connect = on_connect
client.connect("broker.emqx.io",1883,60)

while True:
    inp = input("Enter your message :")
    client.publish("testClient",payload = inp, qos=0, retain=False)
    print("Send to testClient")
    
client.loop_forever()

Subscribe

import context
import paho.mqtt.client as mqtt
import sqlite3
import datetime

db = "externalTestdb.db"
conn = sqlite3.connect(db)
curr = conn.cursor()


def on_connect(client,userdata,flags,rc):
    print("connected with result code" ,{rc})
    client.subscribe("test/publish")
    
def on_message(client,userdata,msg):
    message = msg.payload.decode("utf-8")
    print("Topic :",msg.topic ,"Message :",message)
    curr.execute("create table if not exists tblmqtt (id integer primary key autoincrement,message text,currentdate current_timestamp)")
    sql1="""insert into tblmqtt(message,currentdate) values(?,?)"""
    sql_data=(message,datetime.datetime.now())
    curr.execute(sql1,sql_data)
    conn.commit()
    conn.close()
    
    
    #print("ddata")
#ddata = msg.payload.decode("utf-8")
client = mqtt.Client();
client.on_connect = on_connect
client.on_message = on_message
client.connect("broker.emqx.io",1883,60)
client.loop_forever()

dht11

import sys
import adafruit_dht
import RPi.GPIO as GPIO
import time
import datetime

GPIO.setmode(GPIO.BCM)
GPIO.setup(14,GPIO.IN)

db = sqlite3.connect("external")
cur = db.cursor()

dhtDevice = adafruit_dht.DHT11(14)
while True:
    try:
        temperature_c = dhtDevice.temperature
        temperature_f = temperature_c * (9/5) + 32
        humidity = dhtDevice.humidity
        sql = """insert into tbldht(temp_c,temp_f,humidity,created_dt) values(?,?,?,?);"""
        data_sql = (temperature_c,temperature_f,humidity,datetime.datetime.now())
        cur.execute(sql,data_sql)
        db.commit()
        print("Temp: {:.1f} F/ {:.1f} C/ Humidity : {}%".format(temperature_f,temperature_c,humidity))
        time.sleep(1)
    except RuntimeError as error:
        print(error.args[0])
        

Uart
import serial
if __name__ == '__main__':
  ser = serial.Serial('dev/ttyACM0',9600,timeout= 1)#for cabled connection
  #ser = serial.Serial('dev/ttyS0',9600,timeout= 1)#for jumper wired connection
  ser.flush()
  while True:
    if ser.in_waiting > 0:
      line = ser.readline().decode('utf-8').rstrip()
      print(line)
      print("Arduino is Connected to Raspberry pi")

Motion
import RPi.GPIO as GPIO
import time
import sys
import sqlite3

GPIO.setwarnings(False);
GPIO.setmode(GPIO.BOARD)
GPIO.setup(18,GPIO.IN)
GPIO.setup(24,GPIO.OUT);

DB_NAM = "motion_db"

conn = sqlite3.connect(DB_NAM)
curs = conn.cursor()

curs.execute('create table IF NOT EXISTS motions(motion_id INTEGER PRIMARY KEY AUTOINCREMENT,mtime TIMESTAMP DEFAULT CURRENT_TIMESTAMP,message VARCHAR(50))');
while True:
    i = GPIO.input(18)
    if i ==0:
        print("NO ANY MOTION");
        time.sleep(1);
    if i ==1:
        curs.execute("INSERT INTO MOTIONS(message) values('motion detected')");
        conn.commit();
        print("motion detected")
        GPIO.output(24,1);
        time.sleep(5);
        GPIO.output(24,0);
        
conn.close()


cpu subscribe
import json
import context
import time
import sqlite3
import paho.mqtt.client as mqtt

broker_url = "broker.emqx.io"
broker_port = 1883

connection= sqlite3.connect("external.db")
cursor=connection.cursor()
def on_message(client,userdata,msg):
    print(msg.topic+" "+str(msg.payload) + "\n")
    decodedData = msg.payload.decode().split(',')
    cursor.execute("INSERT INTO cputb(cpu,ram) VALUES(?,?)",[decodedData[0],decodedData[1]])
    cursor.execute("COMMIT")
        
client = mqtt.Client()
client.on_message = on_message
client.connect(broker_url,broker_port)
client.subscribe("mscit/viraj", qos=0)
client.loop_forever()

-->cpu publish

import context
import time
import paho.mqtt.client as mqtt
import json
import psutil

broker_url="broker.emqx.io"
broker_port= 1883
client=mqtt.Client()
client.connect(broker_url,broker_port)

while True:
    try:
        cpuper = psutil.cpu_percent(interval=20)
        cpuram = psutil.virtual_memory().percent
        message = str(cpuper) + "," + str(cpuram)
        print(message)
        client.publish(topic="mscit/viraj",payload=message,qos=0,retain=False)
    except RuntimeError as error:
        print(error,args[0])
        time.sleep(3.0)
        continue
    except Exception as error:
        raise error
    time.sleep(3.0)
==
ThingSpeak:

import paho.mqtt.publish as publish
import psutil
import sqlite3

db = sqlite3.connect("external")
cur = db.cursor()

cur.execute("select * from tblsubscribe")
rows = cur.fetchall()

channelID = "1288685" #change this channelID with your channelID
writeapiKey = "TK90L9OTLBQAH53H" #Change this write API Key with your write API Key
useUnsecuredTCP = False
useSSLWebsockets = True
mqttHost = "mqtt.thingspeak.com"

if useSSLWebsockets:
    import ssl
    tTransport = "websockets"
    tTLS = {'ca_certs':'/etc/ssl/certs/ca-certificates.crt','tls_version' : ssl.PROTOCOL_TLSv1}
    tPort = 443

topic = "channels/" + channelID + "/publish/" + writeapiKey
while(True):
    cpuPer = psutil.cpu_percent(interval=20)
    ramPer = psutil.virtual_memory().percent
    print("CPU= ",  cpuPer, " RAM: ", ramPer)
    for row in rows:
        tPayload = "field1=" + str(row[0])
        try:
            publish.single(topic, payload=tPayload, hostname=mqttHost, port=tPort, tls=tTLS, transport=tTransport)
        except (KeyboardInterrupt):
            break
        except:
            print("There was an error while publishing the data")

============================================humidity_sesor======================================
# This file has been written to your home directory for convenience. It is
# saved as "/home/pi/humidity-2020-11-26-12-02-12.py"

from sense_emu import SenseHat

sense = SenseHat()

green = (0, 255, 0)
white = (255, 255, 255)

while True:
    humidity = sense.humidity
    humidity_value = 64 * humidity / 100
    pixels = [green if i < humidity_value else white for i in range(64)]
    sense.set_pixels(pixels)
