
#ifdef __XC__
#include <xc.h>
#else
#include <htc.h>
//#include <stddef.h>
#endif

#include "IOCycle.h"
#include "DHT22.h"
#include "IOConfig.h"

volatile unsigned char _TMR0;

volatile unsigned char BitCount;
volatile unsigned char WorkingByte;
volatile unsigned char CSum;
volatile unsigned char WorkingCount;
volatile unsigned  char ByteIndex;
volatile unsigned char PreBitCount;



void DHT22Error(void)
{
int loop;
 // KILL interrup
TMR0IE=0;
SetIOChange(CurrentIOPin,0);
CurrentIOCycle=IO_CYCLE_END;
if(CurrentIOPin==0)
    IO0_TRIS=1;
else
    IO1_TRIS=1;
}





void DHT22CycleIdle(void)
{
 unsigned short _temp;

 if (Timerms > DHT22_MS_BETWEEN_CONV) // more than 2000ms (oryginal 1000ms)
 {
   // ok Half second pass 
   // time to start pulse
  
  if(CurrentIOPin == 0)
   {
      IOCANbits.IOCAN0=0;
      IOCAFbits.IOCAF0=0;
      IO0_TRIS=0;
      IO0=0;

   }
   else
  {
      IOCANbits.IOCAN1=0;
      IOCAFbits.IOCAF1=0;
      IO1_TRIS=0;
      IO1=0;
  }
  
    CurrentIOCycle= IO_CYCLE_START;
    BitCount=40;
    WorkingCount=8;
    WorkingByte=0;
    ByteIndex=0;
    PreBitCount=2;
    // set timer0 to 1 us clock
#if _XTAL_FREQ == 16000000
 OPTION_REG	= 0b00000001;	// pullups on, TMR0 @ Fosc/4/4
#else
 // assume 32Mhz
 OPTION_REG	= 0b00000010;	// pullups on, TMR0 @ Fosc/4/8
#endif

    TMR0=0;
    CSum=0;
 }
}

void DHT22CycleStart(void)
{
  // wait until Timerms got sligtly less than 2ms (2 counts).
  if (Timerms > DHT22_MS_BETWEEN_CONV+2)
  {
    // release for 
     if(CurrentIOPin == 0)
       IO0_TRIS=1;
   else
       IO1_TRIS=1;
  

  // wait a little
     asm("NOP");
     asm("NOP");


  

  TMR0=0; // reset Timer 0

 if(CurrentIOPin==0)
 {
     IOCAFbits.IOCAF0=0;
     IOCANbits.IOCAN0=1;
 }
 else 
 {
     IOCAFbits.IOCAF1=0;
     IOCANbits.IOCAN1=1;
 }

 IOCIE=1;

// enable TMR0 interrupt
TMR0IF = 0;
TMR0IE = 1;
CurrentIOStatus=IO_STATUS_UNKNOWN;

  CurrentIOCycle= IO_CYCLE_WAIT;
}
}


void DoDHT22Cycle(void)
{ 
  switch(CurrentIOCycle)
  {
   case IO_CYCLE_WAIT:  break; // do nothing
   case IO_CYCLE_END:   break; // do nothing
   case IO_CYCLE_START: DHT22CycleStart();break;
   default: DHT22CycleIdle();
 }
}

 
 void DHT22IOCAF(void)
{
 

 if(PreBitCount >0)
     PreBitCount--;
else 
 {


   WorkingByte*=2;
   if(_TMR0 > 100)
   WorkingByte|=1;
   WorkingCount--;
   if(WorkingCount==0)
   {
      WorkingCount=8;
      if(ByteIndex < 4)
      {
        CSum += WorkingByte;
        WorkingSensorData.BYTE[ByteIndex++]=WorkingByte;
      }

   }
   BitCount--;
   if(BitCount==0){

    // is Check Sum OK
   if(CSum == WorkingByte)
       CurrentIOStatus=IO_STATUS_OK;
      else
       CurrentIOStatus=IO_STATUS_BAD;

   SetIOChange(CurrentIOPin,0);
   TMR0IE=0;
   CurrentIOCycle=IO_CYCLE_END;
}
}
  

}
