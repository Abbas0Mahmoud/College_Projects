# python modules
import json
import time
from threading import Thread
import paho.mqtt.client as mqtt
import serial.tools.list_ports
import serial

# IoT Cloud Address and Credentials
THINGSBOARD_HOST = 'demo.thingsboard.io'
ACCESS_TOKEN = 'NK69bX7bELeH6q6JdqiU'

# System parameters  data and initials
# System Attributes
TubCapacity_data = {'Tub Capacity': '200 Litters'}
AmmoniaFilterCapacity_data = {'Ammonia Filter Capacity': '20 Litters'}
StorageTankCapacity_data = {'Storage Tank Capacity': '50 Litters'}
SolarPanelMaxVoltage_data = {'Solar Panel Max Voltage': '17.5V'}
SolarPanelMaxCurrent_data = {'Solar Panel Max Current': '4.8A'}
SolarPanelsConnection_data = {'Solar Panels Connection': 'Series'}
SolarPanelsSystemMaxVoltage_data = {'Solar Panels System Max Voltage': '35V'}
SolarPanelsSystemMaxCurrent_data = {'Solar Panels System Max Current': '4.8A'}
BatteryCapacity_data = {'Battery Capacity': '37AH'}

# Solar Panels Data
SolarVoltage_data = {'Solar_Voltage': 0.0}
SolarCurrent_data = {'Solar_Current': 0.0}
SolarPower_data = {'Solar_Power': 0.0}

# Battery data
BatteryVoltage_data = {'Battery_Voltage': 0.0}
BatteryChargingCurrent_data = {'Battery_Charging_Current': 0.0}

# Tub data
TubWaterLevel_data = {'Tub_Water_Level': 0.0}
TubPh_data = {'Tub_PH': 0.0}
TubWaterTemperature_data = {'Tub Water Temperature': 0.0}

# Storage Tank
StorageTankLevel_data = {'Storage Tank Water Level': 0.0}

# Ammonia Filter
AmmoniaFilterLevel_data = {'Ammonia Filter Water Level': 0.0}

# Valves Status for Indication
V1_data = {'Valve1 Status': 0}
V2_data = {'Valve2 Status': 0}
V3_data = {'Valve3 Status': 0}
V4_data = {'Valve4 Status': 0}
V5_data = {'Valve5 Status': 0}
V6_data = {'Valve6 Status': 0}

# Manual Operation Status
# Variable to store manual mode status from cloud
M_data = {'Manual Operation Status': 0}
# Variable to show that it's ok to use manual mode
MI_data = {'Manual Indication Status': False}
# Variable to check if the unit is ready for manual mode
MF_data = {'Manual Flag': False}

# Valves Operation Status
V1op_data = {'Valve1 Operation Status': 0}
V2op_data = {'Valve2 Operation Status': 0}
V3op_data = {'Valve3 Operation Status': 0}
V4op_data = {'Valve4 Operation Status': 0}
V5op_data = {'Valve5 Operation Status': 0}
V6op_data = {'Valve6 Operation Status': 0}


# Serial Communication Configuration
def get_ports():
    ports = serial.tools.list_ports.comports()
    return ports


def findPort(portsFound):
    commPort = 'None'
    numConnection = len(portsFound)

    for i in range(0, numConnection):
        port = foundPorts[i]
        strPort = str(port)

        if 'Silicon Labs CP210x USB to UART Bridge' in strPort:
            splitPort = strPort.split(' ')
            commPort = (splitPort[0])
    return commPort


foundPorts = get_ports()
connectPort = findPort(foundPorts)
ser = serial.Serial(connectPort, 9600)
ser.timeout = 1
print('Connected to ' + connectPort)


def serial_handle():
    INTERVAL = 1
    print("Thread 1 Started")
    next_reading = time.time()
    while True:
        print("Thread 1 Running")
        i = 'on'
        ser.write(i.encode())
        time.sleep(1)
        Message = ser.readline().decode('ascii')
        print(Message)
        StrMessage = str(Message)
        print(StrMessage)
        # Store received data into corresponding parameters
        # Receive Solar Data
        if 'solar' in StrMessage:
            # Convert string message into array
            SplitMessage = StrMessage.split(",")
            print(SplitMessage)
            SolarVoltage_data['Solar_Voltage'] = float(SplitMessage[1])
            print(SolarVoltage_data['Solar_Voltage'])
            SolarCurrent_data['Solar_Current'] = float(SplitMessage[2])
            print(SolarCurrent_data['Solar_Current'])
            SolarPower_data['Solar_Power'] = float(SplitMessage[3])
            print(SolarPower_data['Solar_Power'])
        if 'battery' in StrMessage:
            SplitMessage = StrMessage.split(",")
            print(SplitMessage)
            BatteryVoltage_data['Battery_Voltage'] = float(SplitMessage[1])
            print(BatteryVoltage_data['Battery_Voltage'])
            BatteryChargingCurrent_data['Battery_Charging_Current'] = float(SplitMessage[2])
            print(BatteryChargingCurrent_data['Battery_Charging_Current'])
        if 'tub' in StrMessage:
            SplitMessage = StrMessage.split(",")
            print(SplitMessage)
            TubWaterLevel_data['Tub_Water_Level'] = float(SplitMessage[1])
            print(TubWaterLevel_data['Tub_Water_Level'])
            TubPh_data['Tub_PH'] = float(SplitMessage[2])
            print(TubPh_data['Tub_PH'])
            TubWaterTemperature_data['Tub Water Temperature'] = float(SplitMessage[3])
            print(TubWaterTemperature_data['Tub Water Temperature'])
            StorageTankLevel_data['Storage Tank Water Level'] = float(SplitMessage[4])
            print(StorageTankLevel_data['Storage Tank Water Level'])
        if 'valves' in StrMessage:
            SplitMessage = StrMessage.split(",")
            print(SplitMessage)
            V1_data['Valve1 Status'] = int(SplitMessage[1])
            print(V1_data['Valve1 Status'])
            V2_data['Valve2 Status'] = int(SplitMessage[2])
            print(V2_data['Valve2 Status'])
            V3_data['Valve3 Status'] = int(SplitMessage[3])
            print(V3_data['Valve3 Status'])
            V4_data['Valve4 Status'] = int(SplitMessage[4])
            print(V4_data['Valve4 Status'])
            V5_data['Valve5 Status'] = int(SplitMessage[5])
            print(V5_data['Valve5 Status'])
            V6_data['Valve6 Status'] = int(SplitMessage[6])
            print(V6_data['Valve6 Status'])
        next_reading += INTERVAL
        sleep_time = next_reading - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)


def ManualMode():
    INTERVAL = 1
    print("Thread 3 Started")
    next_reading = time.time()
    while True:
        if M_data['Manual Operation Status'] == True and MF_data['Manual Flag'] == False:
            ser.write('mon'.encode())
            time.sleep(1)
            X = ser.readline().decode('ascii')
            print(X)
            if X == 'start':
                MF_data['Manual Flag'] = True
                MI_data['Manual Indication Status'] = 1
                print('Manual Mode Activated ')
            else:
                print('Manual Mode Not Activated')
        if M_data['Manual Operation Status'] == False and MF_data['Manual Flag'] == True:
            ser.write('moff'.encode())
            time.sleep(1)
            X = ser.readline().decode('ascii')
            print(X)
            if X == 'stop':
                MF_data['Manual Flag'] = False
                MI_data['Manual Indication Status'] = 0
                print('Manual Mode Deactivated ')
            else:
                print('Manual Mode Not Deactivated')
        next_reading += INTERVAL
        sleep_time = next_reading - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)


# client side telemetry function,
# initialize connection between client and server and if connection is established,
# data will be published to the cloud
def publishValue(GateWay):
    INTERVAL = 1
    print("Thread 2 Started")
    next_reading = time.time()
    while True:
        print("Thread 2 Running")
        # Telemetry Messages
        # Solar Panels Parameters
        GateWay.publish('v1/devices/me/telemetry', json.dumps(SolarVoltage_data), 1)
        GateWay.publish('v1/devices/me/telemetry', json.dumps(SolarCurrent_data), 1)
        GateWay.publish('v1/devices/me/telemetry', json.dumps(SolarPower_data), 1)
        # Battery Parameters
        GateWay.publish('v1/devices/me/telemetry', json.dumps(BatteryVoltage_data), 1)
        GateWay.publish('v1/devices/me/telemetry', json.dumps(BatteryChargingCurrent_data), 1)
        # Tub Parameters
        GateWay.publish('v1/devices/me/telemetry', json.dumps(TubWaterLevel_data), 1)
        GateWay.publish('v1/devices/me/telemetry', json.dumps(TubWaterTemperature_data), 1)
        GateWay.publish('v1/devices/me/telemetry', json.dumps(TubPh_data), 1)
        # Storage Tank Parameters
        GateWay.publish('v1/devices/me/telemetry', json.dumps(StorageTankLevel_data), 1)
        # Valves Status
        GateWay.publish('v1/devices/me/telemetry', json.dumps(V1_data), 1)
        GateWay.publish('v1/devices/me/telemetry', json.dumps(V2_data), 1)
        GateWay.publish('v1/devices/me/telemetry', json.dumps(V3_data), 1)
        GateWay.publish('v1/devices/me/telemetry', json.dumps(V4_data), 1)
        GateWay.publish('v1/devices/me/telemetry', json.dumps(V5_data), 1)
        GateWay.publish('v1/devices/me/telemetry', json.dumps(V6_data), 1)
        # Attribute Messages
        GateWay.publish('v1/devices/me/attributes', json.dumps(V1_data), 1)
        GateWay.publish('v1/devices/me/attributes', json.dumps(V2_data), 1)
        GateWay.publish('v1/devices/me/attributes', json.dumps(V3_data), 1)
        GateWay.publish('v1/devices/me/attributes', json.dumps(V4_data), 1)
        GateWay.publish('v1/devices/me/attributes', json.dumps(V5_data), 1)
        GateWay.publish('v1/devices/me/attributes', json.dumps(V6_data), 1)
        GateWay.publish('v1/devices/me/attributes', json.dumps(MI_data), 1)
        next_reading += INTERVAL
        sleep_time = next_reading - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)


# Function to Set Manual Mode for the unit to control it from IoT Platform
def SetManual(params):
    M_data['Manual Operation Status'] = params
    print("Rx SetManual is : ", M_data)
    print("Manual Set : ", params)


# MQTT on_connect callback function
def on_connect(GateWay, userdata, flags, rc):
    print("rc code", rc)
    GateWay.subscribe('v1/devices/me/rpc/request/+')


# MQTT on_message callback function
def on_message(GateWay, userdata, msg):
    print('Topic: ' + msg.topic + '\nMessage: ' + str(msg.payload))
    if msg.topic.startswith('v1/devices/me/rpc/request/'):
        requestId = msg.topic[len('v1/devices/me/rpc/request/'):len(msg.topic)]
        print("requestId : ", requestId)
        data = json.loads(msg.payload)
        if data['method'] == 'SetManual':
            print("SetManual request\n")
            params = data['params']
            SetManual(params)


# Create a client instance
GateWay = mqtt.Client()
GateWay.on_connect = on_connect
GateWay.on_message = on_message
GateWay.username_pw_set(ACCESS_TOKEN)
GateWay.connect(THINGSBOARD_HOST, 1883, 60)

# System Threads
T1 = Thread(target=serial_handle, args=())
T2 = Thread(target=publishValue, args=(GateWay,))
T3 = Thread(target=ManualMode, args=())

try:
    # Read Serial Data Thread
    T1.start()
    # Publish Data to Cloud Thread
    T2.start()
    T3.start()
    GateWay.loop_start()

except:
    print("error occurred")
