#!/usr/bin/python3.2
import minimalmodbus
import time


class PicMbus:
   

  #CONFIGURATION IO

  IOCONFIG_ANALOGVDD= 0  
  IOCONFIG_ANALOG1V=  1 
  IOCONFIG_ANALOG2V=  2  
  IOCONFIG_ANALOG4V=  3  
  IOCONFIG_INPUT=     4
  IOCONFIG_INPUT_PULLUP= 5
  IOCONFIG_OUTPUT=    6
  IOCONFIG_CAP_SENSE_OFF= 8
  IOCONFIG_CAP_SENSE_LOW=9
  IOCONFIG_CAP_SENSE_MEDIUM=10
  IOCONFIG_CAP_SENSE_HIGH=11
  IOCONFIG_DHT11=     16
  IOCONFIG_DHT22=     17
  IOCONFIG_DS18B20=   32
  IOCONFIG_SERVO=     64
  IOCONFIG_COUNTER=   128
  IOConfig=[IOCONFIG_INPUT,IOCONFIG_INPUT]

  def __init__(self,SlaveAddress,Baud=57600,Device='/dev/serial0'):
    self.SlaveAddress= SlaveAddress
    self.module = minimalmodbus.Instrument(Device,SlaveAddress)
    self.module.serial.baudrate=Baud
    self.module.serial.timeout=0.02
    self.module.serial.flushInput();
    #ok lire Configuration
    self.IOConfig[0] = self.module.read_register(0x100,0,3)
    self.IOConfig[1] = self.module.read_register(0x101,0,3)


  def config(self,Pin,value):
    self.module.write_register(0x100+Pin,value,0,6)
    self.IOConfig[Pin]= value

  def readConfig(self,Pin):
    ioconfig=self.module.read_register(0x100+Pin,0,3)
    self.IOConfig[Pin]=ioconfig
    return ioconfig    

  def readVRef2V(self):
    return self.module.read_register(32,0,4)

  def readDiode(self):
    return self.module.read_register(33,0,4)

  def readSensor(self,Pin):
    Mask =  self.IOCONFIG_DHT11 | \
           self.IOCONFIG_DS18B20       | self.IOCONFIG_COUNTER           
    ioconfig= self.IOConfig[Pin]
    if (ioconfig & self.IOCONFIG_CAP_SENSE_OFF)>0:
      return self.module.read_long(Pin*16,4,False)

    elif (ioconfig & (self.IOCONFIG_DHT11 | self.IOCONFIG_DS18B20 | self.IOCONFIG_COUNTER))>0:
      return self.module.read_registers(Pin*16,3,4)
    else:
      return self.module.read_registers(Pin*16,1,4)[0]

  def resetCounter(self,Pin):
     return self.module.write_register(Pin,0,0,6)

  def RCServo(self,Pin,value):
     return self.module.write_register(Pin,value,0,6)


  def readVersion(self):
     version = self.module.read_register(250,0,3)
     print("{}.{}".format(version>>8, version & 0xff))


  def readId(self):
     return self.module.read_register(251,0,3)

  def readIO(self,Pin):
     return self.module.read_bit(Pin)

  def writeIO(self,Pin,Value):
     self.module.write_bit(Pin,Value)


  def readDHT(self,Pin):
     #check config
     if (self.IOConfig[Pin] &  self.IOCONFIG_DHT11) == 0:
        #return IO  config not set for dht
        return None
     while(True):
        value = self.readSensor(Pin)
        if value[0] == 0xffff:
           #No sensor Found
           return None
        if value[0] == 1:
           if self.IOConfig[Pin] == self.IOCONFIG_DHT11 : 
              Factor = 1.0
           else:
              if(value[2] & 0x8000)!=0:
                Factor = (-0.1)
              else:
                Factor  = 0.1
           temperature = (value[2] & 0x7fff) * Factor
           humidity    = value[1] * 0.1 
           return [humidity , temperature]


  def readDS18B20(self,Pin):
     #check config
     if (self.IOConfig[Pin] & self.IOCONFIG_DS18B20) ==0:
        #return IO  config not set for dht
        return None
     while(True):
        value = self.readSensor(Pin)
        if value[0] == 0xffff:
           #No Sensor Found
           return None
        Factor = 0.0625
        Temp = value[1]
        if(Temp & 0x8000) !=0:
           Temp-=65536
        return Temp*Factor

