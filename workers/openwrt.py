#!/usr/bin/env python3
#import json
#import logger
#import inspect
# LINE 64 -> make it so that it only runs if there are new devices, when compared to a device list already sent to home assistant
import json
#from contextlib import contextmanager

#from mqtt import MqttMessage
#from workers.base import BaseWorker

#_LOGGER = logger.get(__name__)

#REQUIREMENTS = ["bluepy"]


### THESE MIGHT BE NEEDED
#from mqtt import MqttConfigMessage

#from mqtt import MqttMessage
#from workers.base import BaseWorker

import os
import subprocess
        

#HA
#from interruptingcow import timeout
#import time
#from exceptions import DeviceTimeoutError
#from mqtt import MqttConfigMessage

ATTR_LOW_BATTERY = 'low_battery'

class rawdata():
    mac: str
    temperature: float
    humidity: float
    counter: int
    battery_mv: int
    battery_pct: int
    rssi: int
    

class results():
    addr: str
    content: str
    rssi: int
 
    def getScanData(self):
        return [("","",self.content)]    
    

class Lywsd03MmcWorker():
    callingclass = None
    blocklist = []
    friendly_name = {}
    command_timeout = 30
    measurement_interval = 4
    rolling_average_count = 8
    topic_prefix = "testtopicprefix"
    global_topic_prefix = "globaltopicprefix"


    devicesDiscovered = [] # home assistant
    monitoredAttrs = ["temperature","temperature_raw", "humidity", "lqi", "packet_loss", "rssi"] #["temperature", "temperature_raw", "humidity","humidity_raw", "battery_pct", "rssi", "battery_mv", "lqi_measurement", "lqi", "packet_loss", "lqi_average"] #add all supported
    def _setup(self): 
        self.devices = {}
        print("Adding devices by auto-discovery. LYWSD03MMC is supported with custom firmware (ATC or pvvx).")
        #self.devices
      #  if hasattr(self, 'monitored_attributes'):
      #    _LOGGER.info("%s",type(self.monitored_attributes))
     #     _LOGGER.info("%s",self.monitored_attributes)
          #for name in self.monitored_attributes.items():
 
       # if hasattr(self, 'friendly_name'):
        #  _LOGGER.info("%s",type(self.friendly_name))
          #_LOGGER.info("%s",self.friendly_name)

   #   stack = inspect.stack()
    #    self.callingclass = stack[2][0]
        #the_class = stack[2][0].f_locals["self"].__class__.__name__
        #the_method = stack[2][0].f_code.co_name
      #  _LOGGER.info("I was called by {}.{}()".format(stack[1][0], stack[1][1]))
   #     _LOGGER.info("I was called by {}.{}()".format(stack[2][0], stack[2][1]))
    #    _LOGGER.info("I was called by {}.{}()".format(stack[3][0], stack[3][1]))
        

#for name, mac in self.devices.items():
            #_LOGGER.info("Adding %s device '%s' (%s)", repr(self), name, mac)
            #self.devices[name] = lywsd03mmc(mac.lower(), command_timeout=self.command_timeout, passive=self.passive, measurement_interval=self.measurement_interval if hasattr(self, 'measurement_interval') else 4)










    def format_discovery_topic(self, mac, *sensor_args):
        node_id = mac.replace(":", "-")
        object_id = "_".join([repr(self), *sensor_args])
        return "{}/{}".format(node_id, object_id)

    def format_discovery_id(self, mac, *sensor_args):
        return "bt-mqtt-gateway/{}".format(
            self.format_discovery_topic(mac, *sensor_args)
        )

    def format_discovery_name(self, *sensor_args):
        return "_".join([repr(self), *sensor_args])

    def format_topic(self, *topic_args):
        return "/".join([self.topic_prefix, *topic_args])

    def format_prefixed_topic(self, *topic_args):
        topic = self.format_topic(*topic_args)
        if self.global_topic_prefix:
            return "{}/{}".format(self.global_topic_prefix, topic)
        return topic

    def __repr__(self):
        return self.__module__.split(".")[-1]



 









    def find_device(self, mac):
        for name, device in self.devices.items():
            if device.mac == mac:
                return device
        return

    def find_supported_device(self, scanData, deviceName, mac):
        for (adtype, desc, data) in scanData:
#            _LOGGER.info("yes?  %s  or (%s)", data[0:4], (len(data) == ((19-2)*2)))

            supported = ((len(data) == ((16-2)*2) or len(data) == ((19-2)*2)) and (data[0:4] == "1a18") )
   
            for mac_part in self.blocklist:
              if mac.lower().replace(":","")[6:12] in str(mac_part).lower().replace(":",""): return False
        #    if not mac.lower().replace(":","")[6:12] in str("B3B4D0").lower().replace(":",""): return False

            if supported and not deviceName in self.devices and not deviceName in self.blocklist:
           #   _LOGGER.info("Adding newly found device: %s (%s)", deviceName, mac)
              pvvx = len(data) == ((19-2)*2)
              _friendly_name = deviceName
              for mac_part, friendlyname in self.friendly_name.items():
                if mac.lower().replace(":","")[6:12] in str(mac_part).lower().replace(":",""):
                  _friendly_name = friendlyname
                 # _LOGGER.info("Found friendly name: %s %s", mac_part, friendlyname )
             # _LOGGER.info("Adding newly found device: %s %s (%s)", _friendly_name, deviceName, mac)
              print("adding supported device!")
              self.devices[deviceName] = lywsd03mmc(mac, command_timeout=self.command_timeout, pvvx=pvvx, friendly_name=_friendly_name, measurement_interval = self.measurement_interval, rolling_average_count = self.rolling_average_count )
              self.config_device(_friendly_name, mac)
              #self.callingclass.f_locals["self"].__class__._publish_config(self.callingclass.f_locals["self"])
            return ((len(data) == ((16-2)*2) or len(data) == ((19-2)*2)) and (data[0:4] == "1a18") )


    #def add_supported_device(self, deviceName, mac):
    #    _LOGGER.info("Adding %s device (%s)", repr(self), mac)
    #    self.devices[deviceName] = lywsd03mmc(mac, command_timeout=self.command_timeout, pvvx)
    

 

    def status_update(self, results):
#        from bluepy import btle
 #       scanner = btle.Scanner()
      #  results = scanner.scan(self.scan_timeout if hasattr(self, 'scan_timeout') else 20.0, passive=True)
   #     print("whyy")
        for res in results:
            mac = res.addr
            deviceName = "ATC_" + mac.upper().replace(":","")[6:12]
            deviceScanData = res.getScanData()
            #device = self.find_device(res.addr)
            found = self.find_supported_device(deviceScanData, deviceName, mac)
            for (adtype, desc, value) in deviceScanData:
              #_LOGGER.debug("Found device %s: (%s) - %s  len: %s", mac, found, value, len(value) )
              if found: 
                device = self.devices[deviceName]
                if (len(value) == 16):
                  device.pvvx = false
               # _LOGGER.info("%s - received scan data %s, RSSI: %s", res.addr, value, res.rssi)
                device.setRSSI(res.rssi)
                device.processScanValue(value) 
      #  print("updating device state")
        if 'device' in locals():
          self.update_device_state(device.friendly_name, device)
     #   yield self.update_device_state(device.friendly_name, device)

        #for name, device in self.devices.items():
        #    if device.freshresults == False: continue
        #    try:
        #        with timeout(self.command_timeout, exception=DeviceTimeoutError):
        #            yield self.update_device_state(device.friendly_name, device)
        #    except btle.BTLEException as e:
        #        logger.log_exception(
        #            _LOGGER,
        #            "Error during update of %s device '%s' (%s): %s",
        #            repr(self),
        #            name,
        #            device.mac,
        #            type(e).__name__,
        #            suppress=True,
        #        )
        #    except DeviceTimeoutError:
        #        logger.log_exception(
        #            _LOGGER,
        #            "Time out during update of %s device '%s' (%s)",
        #            repr(self),
        #            name,
        #            device.mac,
        #            suppress=True,
        #        )
            
## NON-HA
            #try:
            #    ret = device.readAll()
            #except btle.BTLEDisconnectError as e:
            #    self.log_connect_exception(_LOGGER, name, e)
            #except btle.BTLEException as e:
            #    self.log_unspecified_exception(_LOGGER, name, e)
            #else:
            #    yield [MqttMessage(topic=self.format_topic(name), payload=json.dumps(ret))]


# Home assistant integration
    def config(self, availability_topic):

        #stack = inspect.stack()
        
   #     _LOGGER.info("CONFIG was called by {}.{}()".format(stack[1][0], stack[1][1]))
    #    _LOGGER.info("CONFIG was called by {}.{}()".format(stack[2][0], stack[2][1]))
   #    _LOGGER.info("CONFIG was called by {}.{}()".format(stack[3][0], stack[3][1]))
   #  # //  _LOGGER.info("is this working?? CONFIG " )
       # ret = []
        #ret += device
     #   for name, device in self.devices.items():
     #       ret += self.config_device(name, device.mac)
        return self.devicesDiscovered

# Home assistant integration
    def config_device(self, name, mac):
        ret = []
        device = {
            "identifiers": [mac, self.format_discovery_id(mac, name)],
            "manufacturer": "Xiaomi",
            "model": "Mijia Lywsd03Mmc",
            "name": self.format_discovery_name(name),
        }
        for attr in self.monitoredAttrs:
            payload = {
                "unique_id": self.format_discovery_id(mac, name, attr),
                "state_topic": self.format_prefixed_topic(name, attr),
                "name": self.format_discovery_name(name, attr),
                "device": device,
            }

            if attr == "humidity":
                payload.update({"icon": "mdi:water", "unit_of_measurement": "%"}) 
            elif attr == "humidity_raw":
                payload.update({"icon": "mdi:water-percent", "unit_of_measurement": "%"})
            elif attr == "temperature":
                payload.update({"device_class": "temperature", "unit_of_measurement": "°C"})
            elif attr == "temperature_raw":
                payload.update({"icon": "mdi:thermometer-lines", "unit_of_measurement": "°C"})
            elif attr == "battery_pct":
                payload.update({"icon": "mdi:battery", "unit_of_measurement": "%"})
             #   payload.update({"device_class": "battery", "unit_of_measurement": "%"})
            elif attr == "battery_mv":
                payload.update({"device_class": "voltage", "unit_of_measurement": "mV"})
            elif attr == "lqi_measurement":
                payload.update({"icon": "mdi:broadcast", "unit_of_measurement": "lqi²"})
            elif attr == "lqi":
                payload.update({"icon": "mdi:signal", "unit_of_measurement": "lqi"})
            elif attr == "lqi_average":
                payload.update({"icon": "mdi:signal", "unit_of_measurement": "lqi"})
            elif attr == "rssi":
                payload.update({"icon": "mdi:access-point", "unit_of_measurement": "rssi"})
            elif attr == "packet_loss":
                payload.update({"icon": "mdi:broadcast", "unit_of_measurement": "%"})
            jsonpayload = json.dumps(payload)
#            print(payload)
            entirecommand = "/usr/bin/mosquitto_pub -h ut29.wondersense.fi -p 21218 -u mqttsensor -P apuaanoimqtt -t homeassistant/sensor/" + mac + "/" + attr + "/config " + "-m"
            progcommand = "/usr/bin/mosquitto_pub"
            address = "-h ut29.wondersense.fi"
            port =  "-p 21218 "
            user = "-u mqttsensor " 
            password = "-P apuaanoimqtt "
            topic = "-t homeassistant/sensor/" + mac + "/" + attr + "/config "
            value = "-m payload"
            message = "\"" + jsonpayload.replace("\"", "\\\""  ) + "\"" 
            message =  jsonpayload
         #   print (message)
            allstuff = entirecommand.split(" ")
            allstuff.append(message)
       #     subprocess.Popen([progcommand, address, port, user, password,  topic,  value])            
            subprocess.Popen(allstuff)
#homeassistant/sensor/
            #ret.append(
            #    mqttconfigmessage(
            #        mqttconfigmessage.sensor,
            #        self.format_discovery_topic(mac, name, attr),  #definition in base.py
            #        payload=payload,
            #    )
            #)

        #ret.append(
        #    MqttConfigMessage(
        #        MqttConfigMessage.BINARY_SENSOR,
        #        self.format_discovery_topic(mac, name, ATTR_LOW_BATTERY), #definition in base.py
        #        payload={
        #            "unique_id": self.format_discovery_id(mac, name, ATTR_LOW_BATTERY),
        #            "state_topic": self.format_prefixed_topic(name, ATTR_LOW_BATTERY),
        #            "name": self.format_discovery_name(name, ATTR_LOW_BATTERY),
        #            "device": device,
        #            "device_class": "battery",
        #        },
        #    )
        #)

        #for command in self._config_commands:
        #    messages = command.execute()
        #    for msg in messages:
        #        msg.topic = "{}/{}".format(
        #            self._config["sensor_config"].get("topic", "homeassistant"),
        #            msg.topic,
        #        )
        #        msg.retain = self._config["sensor_config"].get("retain", True)
        #    self._mqtt.publish(messages)


        #self.devicesDiscovered += ret
        self.devicesDiscovered = ret
       # return ret

# Home assistant integration
    def update_device_state(self, name, device):
        ret = []
     #   _LOGGER.info("Updating state")
   #     if device.readAll() is None :
     #       return ret
        for attr in self.monitoredAttrs:
            attrValue = None
            if attr == "humidity":
                attrValue = device.getHumidity()
            if attr == "humidity_raw":
                attrValue = device.getHumidityRaw()
            elif attr == "temperature":
                attrValue = device.getTemperature()
            elif attr == "temperature_raw":
                attrValue = device.getTemperatureRaw()
            elif attr == "battery_pct":
                attrValue = device.getBatteryPercentage()
            elif attr == "battery_mv":
                attrValue = device.getBatteryVoltage()
            elif attr == "rssi":
                attrValue = device.getRSSI()
            elif attr == "lqi_measurement":
                attrValue = device.getLQImeas()
            elif attr == "lqi":
                attrValue = device.getLQI()
            elif attr == "packet_loss":
                attrValue = device.getPacketloss()
            elif attr == "lqi_average":
                attrValue = device.getLQIAverage()

            if attrValue == "skip": continue
            #print("%s: %s", topic, attrValue)
            #jsonpayload = json.dumps(payload)
            entirecommand = "/usr/bin/mosquitto_pub -h ut29.wondersense.fi -p 21218 -u mqttsensor -P apuaanoimqtt -t " +  self.format_prefixed_topic(name, attr) + " -m"

            message =  str(attrValue)
      #      print (message)
            allstuff = entirecommand.split(" ")
            
            allstuff.append(message)
       #     subprocess.Popen([progcommand, address, port, user, password,  topic,  value])            
            subprocess.Popen(allstuff)


            #ret.append(
            #    MqttMessage(
            #        topic=self.format_topic(name, attr),
            #        payload=attrValue,
            #    )
            #)

        #ret.append(
        #    MqttMessage(
        #        topic=self.format_topic(name, ATTR_LOW_BATTERY),
        #        payload=self.true_false_to_ha_on_off(device.getBatteryPercentage() < 3),
        #    )
        #)

        return ret


class lywsd03mmc:
    def __init__(self, mac,friendly_name, command_timeout=30, pvvx = False, measurement_interval = 4, rolling_average_count = 8 ):
        self.mac = mac
        self.command_timeout = command_timeout
        self.pvvx = pvvx
        self.freshresults = True
        self.measurement_interval = measurement_interval
        self.friendly_name = friendly_name
        self.rolling_average_count = rolling_average_count

        self._last_updated = -999 ## implement
        self._temperature_raw = None 
        self._humidity_raw = None
        self._battery_pct = None
        self._battery_mv = None
        self._rssi = 0
        self._previousCounter = 0
        #self._currentCounter = None

        self._LQI=[0 for i in range(256)] 
        self._received_packets=[0 for i in range(256)]
        self._TemperatureHistory=[0.0 for i in range(256)]
        self._HumidityHistory=[0.0 for i in range(256)]

      #  self._devicecounter = 0 #total count
        self._TemperatureHistoryLastUpdate = -99  
        self._HumidityHistoryLastUpdate = -99

        self.Measurements={}
        self.Measurements["received_packets"] = Measurements(self.rolling_average_count)
        self.Measurements["LQI"] = Measurements(self.rolling_average_count, 0, self.Measurements["received_packets"])
        self.Measurements["temperature"] = Measurements(self.rolling_average_count, 0.03, self.Measurements["received_packets"])
        self.Measurements["humidity"] = Measurements(self.rolling_average_count, 0.05, self.Measurements["received_packets"])
        
    #def readAll(self):
    #    temperature = self.getTemperature()
    #    temperature_raw = self.getTemperatureRaw()
    #    humidity = self.getHumidity()
    #    humidity_raw = self.getHumidityRaw()
    #    battery_pct = self.getBatteryPercentage()
    #    battery_mv = self.getBatteryVoltage()
    #    rssi = self.getRSSI()
    #    lqi_meas = self.getLQImeas()
    #    lqi = self.getLQI()
    #    packetloss = self.getPacketloss()
    #    lqi_average = self.getLQIAverage()

        

    #    if temperature_raw and humidity:
    #        _LOGGER.debug("%s - data received ok", self.mac)#_LOGGER.debug("%s - found values %f C, %d %%, %d, RSI: %d, LQI: %d, count: %d, totalcount: %d ", self.mac, temperature, humidity, battery, rssi, linkqualityin, count, self. self._devicecounter)
    #    else:
    #        _LOGGER.debug("%s - no data received", self.mac)

    #    return {
    #        "temperature_raw": temperature_raw,
    #        "temperature": temperature,
    #        "humidity_raw": humidity_raw,
    #        "humidity": humidity,
    #        "battery_pct": battery_pct,
    #        "battery_mv": battery_mv,
    #        "rssi": rssi,
    #        "lqi_measurement": lqi_meas,
    #        "lqi": lqi,
    #        "packetloss": packetloss,
    #        "lqi_average": lqi_average
    #    }

    def getTemperature(self):
        return self.Measurements["temperature"].getValue()

    def getHumidity(self):
        return self.Measurements["humidity"].getValue()

    def getTemperatureRaw(self):
        return self.Measurements["temperature"].getValue(True)
    
    def getHumidityRaw(self):
        return self.Measurements["humidity"].getValue(True)

    def getHumidity(self):
        return self.Measurements["humidity"].getValue()

    def getBatteryPercentage(self):
        return self._battery_pct
    
    def getBatteryVoltage(self):
        return self._battery_mv
        
    def getRSSI(self):
        return self._rssi

    def setRSSI(self, value):
         self._rssi = int(value)
        
    def getLQImeas(self):
        if self.Measurements["received_packets"].device_counter >= 255:
           return sum(self.Measurements["received_packets"].records) - 1; #all packets received is 256/256. Substracting 1 so that LQI is max 255. Equiv. with Zigbee specs.
        return 'unavailable'

    def getPacketloss(self):
      #  _LOGGER.info(sum(self.Measurements["received_packets"].records))
        if self.Measurements["received_packets"].device_counter >= 255:
           return round(100 - (sum(self.Measurements["received_packets"].records)*20//256)/20*100);
        return 'unavailable'

    def getLQIAverage(self):
        if self.Measurements["received_packets"].device_counter >= 255:
           return round(sum(self.Measurements["LQI"].records)/self.measurement_interval) - 1;
        return 'unavailable'
    
    def getLQI(self):
        firstCounter = self._previousCounter - 64
       # _LOGGER.info("counter: %s", self.Measurements["received_packets"].device_counter )
        #_LOGGER.info("%s - measuement: %s --  %s", self.mac, sum(self.Measurements["LQI"].records) ,' '.join(str(x) for x in self.Measurements["LQI"].records))
     #   _LOGGER.info("current counter: %s .. devcounter: %s.. %s - measuement: %s ", self._previousCounter, self.Measurements["received_packets"].device_counter, sum(self.Measurements["LQI"].records[max(firstCounter,0):self._previousCounter]), sum(self.Measurements["LQI"].records[255+min(firstCounter,0) : 255]))

        if self.Measurements["received_packets"].device_counter >= 63:
           firstCounter = self._previousCounter - 64
                   
           return sum(self.Measurements["LQI"].records[max(firstCounter,0):self._previousCounter]) + sum(self.Measurements["LQI"].records[255+min(firstCounter,0) : 255])
        return 'unavailable'



    def subscribe(self, device):
        device.setDelegate(self)

    def reverseMAC(self, rev):
        rev = ""
        for x in range(-1, -len(a), -2):
          rev += a[x-1] + a[x]
        return value

    def toLittleEndian(self, a, signed):
        ba = bytearray.fromhex(a)
        value = int.from_bytes(ba, byteorder='little', signed=signed)
        return value


    def processScanValue(self, data):
      #  _LOGGER.info("Processing scan values")
        if self.pvvx:
            #_LOGGER.info("From value: %s", data)
            temperature_raw = self.toLittleEndian(data[16:20], True) / 100 #int.from_bytes(data[16:20], byteorder='little', signed=True) / 100 
            humidity_raw = self.toLittleEndian(data[20:24], False) /100
            battery_mv = self.toLittleEndian(data[24:28], False)
            battery_pct = self.toLittleEndian(data[28:30], False)
            counter = self.toLittleEndian(data[30:32], False)
            flags = self.toLittleEndian(data[32:34], False)
        

            self._temperature_raw = round(temperature_raw,2)
            self._humidity_raw = round(humidity_raw, 2)
            self._battery_pct = round(battery_pct)
            self._battery_mv = round(battery_mv)
            #self.recordCounter(counter)
            

            if counter == self._previousCounter:     # broadcast for same measurement       
              self.freshresults = False
              self.Measurements["LQI"].storeValue(counter, 1, True)
            else:
              self.freshresults = True
              self.Measurements["received_packets"].storeValue(counter, 1) 
           #   _LOGGER.info (sum(              self.Measurements["received_packets"].records))
              self.Measurements["LQI"].storeValue(counter, 1)
              self.Measurements["temperature"].storeValue(counter, self._temperature_raw)
              self.Measurements["humidity"].storeValue(counter, self._humidity_raw)
            self._previousCounter = counter

#class MeasurementsHandler:
#    def __init__(self, rolling_average_count = 8, received_packets = None):
#      self.received_packets = received_packets
#      self.records=[0.0 for i in range(256)]
#    def storeValue(self, counter_position, sensor_value, addition = False):
      


class Measurements:
    def __init__(self, rolling_average_count = 8, hysterisis = 0.03, received_packets = None):
      self.received_packets = received_packets
      self.records=[0.0 for i in range(256)]
      self.previousUpdateCounterPosition=None
      self.previousCounterPosition=None
      self.previousValue=0.0
      self.currentValue=0.0
      self.hysterisis = hysterisis
      self.device_counter=0
      self.rolling_average_count = rolling_average_count

    def storeValue(self, counter_position, sensor_value, addition = False):
        if self.previousCounterPosition != None and self.previousCounterPosition != counter_position:
           self.device_counter += 1
           if counter_position > self.previousCounterPosition: #without rollover

              for i in range(self.previousCounterPosition + 1, counter_position ):  
          #       _LOGGER.info (" zroing: %s --  %s",   i, counter_position )      
                 self.records[i] = 0
                 self.device_counter += 1
           else:  #with rollover
           #   _LOGGER.info (" rollover: %s --  %s",   self.previousCounterPosition + 1, counter_position )
              self.device_counter += 256 - self.previousCounterPosition + counter_position
              if self.previousCounterPosition < 255:
                 for i in range(self.previousCounterPosition + 1, 256):
                    ## .info (" zroing rollover: %s --  previousLoc %s currentLoc %s",   i, self.previousCounterPosition, counter_position )      
                    self.records[i] = 0
              for i in range(0, counter_position+1):
                self.records[i] = 0
        #_LOGGER.info("- write range: %s ", counter_position)
        self.records[counter_position] = self.records[counter_position] + sensor_value if addition else sensor_value
        self.currentValue = sensor_value
        self.previousCounterPosition = counter_position

    def getValue(self, raw = False):
        if raw: return self.currentValue
        if self.received_packets == None: return 'unavailable'

        measurements = {}
        for i in range(3):
            startPosition = self.previousCounterPosition + 1 - self.rolling_average_count - self.rolling_average_count * i
            endPosition = self.previousCounterPosition + 1 - self.rolling_average_count * i 
            measurements[str(i) + "_sum"] = sum(self.records[max(startPosition,0):endPosition]) + sum(self.records[255+min(startPosition,0) : 255])
            measurements[str(i) + "_count"] =sum(self.received_packets.records[max(startPosition,0):endPosition]) + sum(self.received_packets.records[255+min(startPosition,0) : 255])
            measurements[str(i)] =  0 if measurements[str(i) + "_count"] == 0 else round(measurements[str(i) + "_sum"] / measurements[str(i) + "_count"] , 2)
        sameSign = abs(-2*measurements["0"] + measurements["1"] + measurements["2"]) ==   abs(measurements["1"]-measurements["0"] ) + abs(measurements["2"]-measurements["0"])
        measurementDiff = round(abs(self.previousValue - self.currentValue ),2)
        if (not self.previousValue == 0 and measurementDiff == 0.0 or (not sameSign and measurementDiff < self.hysterisis )): ##   or self.previousCounterPosition % 64 == 0 ##Only update if sensor change is consistent (=constantly positive or negative), the difference is larger than 0.03 or every 64 packets (assuming that this packet is received, not lost)
            #_LOGGER.info("Skipping sending. Samesign: %s, diff: %s, value: %s", sameSign, measurementDiff,measurements["0"] )
     #       _LOGGER.info("skipping temp signalling")
            return 'skip'

        else:
          #  _LOGGER.info("ok. Samesign: %s, diff: %s, value: %s", sameSign, measurementDiff,measurements["0"] )
            self.previousValue = self.currentValue
            return measurements["0"]        

worker = Lywsd03MmcWorker()
worker._setup()
#with os.popen('hcidump --raw') as pse:
with os.popen('source ./bletool.sh -python') as pse:
    for line in pse:
        broadcastvalue = line[1:-1].replace(" ", "").lower()
        #mac =broadcastvalue[46:48]+broadcastvalue[44:46]+broadcastvalue[42:44]+broadcastvalue[40:42]+broadcastvalue[38:40]+ broadcastvalue[36:38]
#  mac=${packet:46:2}${packet:44:2}${packet:42:2}${packet:40:2}${packet:38:2}${packet:36:2}
        #print(mac)
        #print(output)
        #continue

        listofresults=[]
        result = results()
        listofresults.append(result)
      #  listofresults[output[0]] = result
        result.addr = broadcastvalue[46:48]+broadcastvalue[44:46]+broadcastvalue[42:44]+broadcastvalue[40:42]+broadcastvalue[38:40]+ broadcastvalue[36:38]
        result.rssi = int.from_bytes(bytearray.fromhex(broadcastvalue[-2:]), byteorder='little', signed=True)
        result.content = broadcastvalue[32:-2]
        

        worker.status_update(listofresults)  
        #worker.status_update(listofresults)    
        #output = line.split("\t")
        #worker.status_update(line)
        #rawoutput = rawdata()
        #rawoutput.mac = output[0]
        #rawoutput.temperature = output[1]
        #rawoutput.humidity= output[2]
        #rawoutput.battery_mv = output[3]
        #rawoutput.battery_pct = output[4]
        #rawoutput.counter = output[5]
        #rawoutput.rssi = output[7]
        #worker.status_update()
