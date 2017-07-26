
#ifdef __XC__
#include <xc.h>
#else
#include <htc.h>
//#include <stddef.h>
#endif



#include "IOConfig.h"
#include "IOCycle.h"
#include "DS18B20.h"
#include "DHT22.h"
#include "CapSense.h"


unsigned char CurrentIOStatus;
unsigned char CurrentIOSensor;
near unsigned char CurrentIOPin;
near unsigned char CurrentIOCycle;

SensorDataUnion  IOSensorData[INPUT_COUNT];
SensorDataUnion  WorkingSensorData;




volatile unsigned char _TMR0;
volatile unsigned short _TMR0_MSB;
volatile unsigned char BitCount;
volatile unsigned char WorkingByte;
volatile unsigned char CSum;
volatile unsigned char WorkingCount;
volatile unsigned  char ByteIndex;
volatile unsigned char PreBitCount;
bit      Timer0Overflow;
bit      ServoTimeOutFlag;
bit      TimerSecFlag;
bit      IO0CounterFlag;
bit      IO1CounterFlag;

unsigned short COUNTER0;
unsigned short COUNTER1;


void DealWithError(void)
{
   TMR0IE=0;
   TMR0IF=0;
   WorkingSensorData.WORD[2]=0xFFFF;
   if(CurrentIOSensor == IOCONFIG_DHT22)
      DHT22Error();
}


void ScanNextIOPin(void)
{
    unsigned char loop;
    CurrentIOPin++;




  if(CurrentIOPin >= INPUT_COUNT)
   CurrentIOPin=0;

  CurrentIOSensor = Setting.IOConfig[CurrentIOPin];
  CurrentIOCycle= IO_CYCLE_IDLE;
  CurrentIOStatus=IO_STATUS_UNKNOWN;
  Timerms=0;
  TMR0IE=0;
  Timer0Overflow=0;

  if(CurrentIOSensor & IOCONFIG_CAP_SENSE_OFF)
      Timer0Overflow=1;

  for(loop=0;loop<SENSOR_DATA_BYTE_MAX;loop++)
      WorkingSensorData.BYTE[loop]=0;

}

void DoIOCycle(void)
{

 SensorDataUnion *SensorPt;

  unsigned char  loop;
  if(CurrentIOCycle==  IO_CYCLE_END)
  {
    if(CurrentIOStatus==IO_STATUS_UNKNOWN)
       DealWithError();

    // Re-organize DATA

    SensorPt = &IOSensorData[CurrentIOPin];

    // DHT22 Nothing to do data is correct


    if(CurrentIOStatus==IO_STATUS_BAD)
        SensorPt->WORD[0]=0xffff;
    else
    {
        SensorPt->BYTE[0]=0;
        SensorPt->BYTE[1]=CurrentIOStatus;
    }

    if(CurrentIOSensor == IOCONFIG_DHT11)
    {
       // OK device is only 8 bits so move MSB to LSB and zero MSB
//       SensorPt->WORD[0]= CurrentIOStatus;
       SensorPt->BYTE[3]=WorkingSensorData.BYTE[0];
       SensorPt->BYTE[5]=WorkingSensorData.BYTE[2];
       SensorPt->BYTE[2]=0;
       SensorPt->BYTE[4]=0;
    }
    else if(CurrentIOSensor == IOCONFIG_DHT22)
    {
       // OK device is only 8 bits so move MSB to LSB and zero MSB
//       SensorPt->WORD[0]= CurrentIOStatus;
       SensorPt->WORD[1]=WorkingSensorData.WORD[0];
       SensorPt->WORD[2]=WorkingSensorData.WORD[1];
    }
    if(CurrentIOSensor == IOCONFIG_DS18B20)
    {
        // Reorganize BYTE
//       SensorPt->WORD[0]= CurrentIOStatus;
       SensorPt->BYTE[2]=WorkingSensorData.BYTE[1];
       SensorPt->BYTE[3]=WorkingSensorData.BYTE[0];
       SensorPt->BYTE[4]=0;
       SensorPt->BYTE[5]=WorkingSensorData.BYTE[4];
    }

    if(CurrentIOSensor & IOCONFIG_CAP_SENSE_OFF)
    {
        // CAPSENSE
        SensorPt->DWORD=WorkingSensorData.DWORD;
    }


    // now it is time for next sensor;
         ScanNextIOPin();
 

 }

 if(CurrentIOCycle < IO_CYCLE_WAIT)
  {
    if(CurrentIOSensor & IOCONFIG_DHT11)
      DoDHT22Cycle();
    else if(CurrentIOSensor & IOCONFIG_DS18B20)
        DoDS18B20Cycle();
    else if(CurrentIOSensor & IOCONFIG_CAP_SENSE_OFF)
        DoCapSenseCycle();
    else
     {
        // sensor doesn't need cycle  jump to the next one
        ScanNextIOPin();
     }
   }

}

/*
void DealWithIOCAF(void )
{
    volatile unsigned char _temp;

    _temp = IOCAF & IOCAN;

    if(_temp&1)
    {
        //IO0
        IOCAFbits.IOCAF0=0;

        if(IO0CounterFlag)
        {
            IOSensorData[0].DWORD++;
            COUNTER0++;
        }
        else if(CurrentIOSensor & IOCONFIG_DHT11)
            DHT22IOCAF();
    }
    if(_temp&2)
    {
        //IO1
        IOCAFbits.IOCAF1=0;

        if(IO1CounterFlag)
        {
            IOSensorData[1].DWORD++;
            COUNTER1++;
        }
        else if(CurrentIOSensor & IOCONFIG_DHT11)
            DHT22IOCAF();
    }
}
*/

void ResetIOCycle(void)
{
 unsigned char loop;
 for(loop=0;loop<INPUT_COUNT;loop++)
 {
    if((Setting.IOConfig[loop]==IOCONFIG_DHT11) ||
       (Setting.IOConfig[loop]==IOCONFIG_DHT22))
        SetIOChange(loop,0);

 CurrentIOCycle= IO_CYCLE_IDLE;
 CurrentIOSensor = Setting.IOConfig[0];
 CurrentIOPin=0;
}
}



void SetIOChange(unsigned char Pin, unsigned char value)
{
    if(Pin==0)
        IOCANbits.IOCAN0= value;
    else
        IOCANbits.IOCAN1= value;
}




