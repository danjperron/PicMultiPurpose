

#ifdef __XC__
#include <xc.h>
#else
#include <htc.h>
//#include <stddef.h>
#endif

#include "IOCycle.h"
#include "DS18B20.h"
#include "IOConfig.h"


void DS18B20CycleIdle(void)
{
 unsigned short _temp;


if(Timerms > 1000)  //more than 1 second
 {
   // ok Half second pass 
   // time to start pulse
  
  if(CurrentIOPin == 0)
   {
      IO0_TRIS=0;
      IO0=0;
   }
  else
  {
       IO1_TRIS=0;
       IO1=0;
  }
  CurrentIOCycle= IO_CYCLE_START;
 }
}

near bit NodeviceFound;

void SetTimer0Delay1us(unsigned char value)
{
   // TMR0 no interrupt
    TMR0IE=0;
   // TMR0 on us clock
#if _XTAL_FREQ == 16000000
        OPTION_REG= 0b00000001;
#else
        // assum 32Mhz
        OPTION_REG= 0b00000010;
#endif
        TMR0 = ~(value)+1;
        TMR0IF=0;
}

void SetTimer0Delay32us(unsigned char value)
{
   // TMR0 no interrupt
    TMR0IE=0;
   // TMR0 on 32us clock
#if _XTAL_FREQ == 16000000
        OPTION_REG= 0b00000110;
#else
        // assum 32Mhz
        OPTION_REG= 0b00000111;
#endif
        TMR0 = ~(value)+1;
        TMR0IF=0;
}



void  DS18B20Reset()
{

    if(CurrentIOPin == 0)
    {
      IO0_TRIS=0;
      IO0=0;
      IOCANbits.IOCAN0=0;
      IOCAFbits.IOCAF0=0;
    }
   else
    {
      IO1_TRIS=0;
      IO1=0;
      IOCANbits.IOCAN1=0;
      IOCAFbits.IOCAF1=0;

   }
    // USE TMR0
    // with 32 us delay
    // delay 480 us
    SetTimer0Delay32us(15);
}


      
   

unsigned char DS18B20ResetCheckForDevice()
{
   if(CurrentIOPin == 0)
        IO0_TRIS=1;
    else
        IO1_TRIS=1;

   // delay of 60us
   SetTimer0Delay1us(60);
   while(!TMR0IF);

    if(CurrentIOPin == 0)
      NodeviceFound=IO0;
    else
      NodeviceFound=IO1;
    if(NodeviceFound)
       return( 0);
    // 480 us delay
    SetTimer0Delay32us(15);
       return(1);
}    
     

unsigned char DS18B20Read()
{
   unsigned char mask=1;
   unsigned char loop;
   unsigned char result=0;

    for(loop=8;loop > 0 ;loop--)
    {
     if(CurrentIOPin == 0)
     {
      IO0_TRIS=0;
      IO0=0;
     }
     else
     {
      IO1_TRIS= 0;
      IO1=0;
     }

     GIE=0;
    SetTimer0Delay1us(1);
     while(!TMR0IF);

     // delay 2us
     //__delay_us(1);


      if(CurrentIOPin == 0)
        IO0_TRIS=1;
      else
        IO1_TRIS= 1;

     // __delay_us(3);
   SetTimer0Delay1us(1);
   while(!TMR0IF);
   
      if(CurrentIOPin ==0)
      {
        if(IO0)
          result |= mask;
      }
      else
      {
        if(IO1)
          result |= mask;
      }
   GIE=1;
      // delay 60us
     SetTimer0Delay1us(60);
     while(!TMR0IF);
     mask <<=1;
    }        
  return result;
}

void DS18B20Write(unsigned char value)
{
  unsigned char loop;
  unsigned char mask=1;
  for(loop=8;loop>0;loop--)
  {
      GIE=0;
    if(CurrentIOPin == 0)
     {
      IO0_TRIS= 0;
      IO0=0;
     }
   else
     {
      IO1_TRIS = 0;
      IO1=0;
     }
  
     if(value & mask)
     {
       SetTimer0Delay1us(1);
       while(!TMR0IF);
       GIE=1;
     }
     else
     {
         GIE=1;
 // delay of 60us
   SetTimer0Delay1us(60);
   while(!TMR0IF);
     }
      

      if(CurrentIOPin == 0)
        IO0_TRIS= 1;
     else
      IO1_TRIS=1;

    if(value & mask)
    {
       //__delay_us(60);
       SetTimer0Delay1us(60);
       while(!TMR0IF);
    }

     
    mask <<=1;
  }
}


void DoDS18B20Cycle(void)
{
 static unsigned char waitLoop;

  switch(CurrentIOCycle)
  {
    case IO_CYCLE_IDLE: 
                          DS18B20CycleIdle();
                          return;
    case IO_CYCLE_START:
                         CurrentIOCycle = IO_CYCLE_DS18B20_START;
    case IO_CYCLE_DS18B20_RESET2:
                         DS18B20Reset();
                         break;
    case IO_CYCLE_DS18B20_RESET_WAIT:  //wait ~480us
    case IO_CYCLE_DS18B20_RESET2_WAIT:
      case IO_CYCLE_DS18B20_RESET_WAIT2:
      case IO_CYCLE_DS18B20_RESET2_WAIT2:
        if(TMR0IF)
            break; //ok time out next
        return;

    case IO_CYCLE_DS18B20_RESET_CHECK_FOR_DEVICE:
    case IO_CYCLE_DS18B20_RESET2_CHECK_FOR_DEVICE:
          if(DS18B20ResetCheckForDevice()==0)
                        {
                         CurrentIOStatus=IO_STATUS_BAD;
                         CurrentIOCycle=IO_CYCLE_END;
                         return;
                        }
                        
                        break;
    case IO_CYCLE_DS18B20_SKIP_ROM:
    case IO_CYCLE_DS18B20_SKIP_ROM2:
                        DS18B20Write(DS18B20_SKIP_ROM);
                        break;
    case IO_CYCLE_DS18B20_CONVERT_T:
                        DS18B20Write(DS18B20_CONVERT_T);
                        waitLoop=125;
                        // set 8 ms delay 32us * 250 = 8 ms
                        SetTimer0Delay32us(250);
                        break;
    case IO_CYCLE_DS18B20_WAIT:// wait ~800 ms for conversion
                        if(!TMR0IF)  // is 8ms timeout  from timer0 done
                           return;
                        waitLoop--;
                        if(waitLoop==0)
                          break;
                        // set 8 ms delay
                        SetTimer0Delay32us(250);
                        return;
     case IO_CYCLE_DS18B20_READ_SCRATCHPAD:
                        DS18B20Write(DS18B20_READ_SCRATCHPAD);
                        ByteIndex=0;
		        break;
    case IO_CYCLE_DS18B20_READ_BYTE:

                        WorkingSensorData.BYTE[ByteIndex]=DS18B20Read();
                        ByteIndex++;
                        if(ByteIndex>5)
                           {
                            CurrentIOCycle=IO_CYCLE_END;
                            CurrentIOStatus=IO_STATUS_OK;
                           }
                       return;           
  } 

   CurrentIOCycle++;    
}



