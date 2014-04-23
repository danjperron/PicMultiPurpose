#ifdef __XC__
#include <xc.h>
#else
#include <htc.h>
//#include <stddef.h>
#endif

#include "IOCycle.h"
#include "DHT22.h"
#include "IOConfig.h"
#include "RCServo.h"

unsigned char ServoIndex;
volatile unsigned short CurrentServoTimer;
unsigned short ServoTimer[INPUT_COUNT];


void RCServoISR(void)
{

if(ServoTimeOutFlag)
{
    if(ServoIndex==0)
        IO0=0;
    else if(ServoIndex==1)
        IO1=0;
}
    ServoIndex++;
    if(ServoIndex>INPUT_COUNT)
    ServoIndex=0;

    TMR1ON=0;
    TMR1IF=0;
    if(ServoIndex==INPUT_COUNT)
        CurrentServoTimer=12000;
    else
        CurrentServoTimer = ServoTimer[ServoIndex];
    if(CurrentServoTimer==0)
    {
    ServoTimeOutFlag=0;
    TMR1= 64536;
    }
    else
    {
        ServoTimeOutFlag=1;
        TMR1 = (~CurrentServoTimer)+1;
    }
    TMR1ON=1;
    if(ServoIndex==0)
        IO0=1;
    else if(ServoIndex==1)
        IO1=1;
}


void DoRCServo(void)
{
    unsigned char loop;
    unsigned char flag=0;
    unsigned short _temp;


    _temp= 16000;
    for(loop=0;loop < INPUT_COUNT;loop++)
        if(Setting.IOConfig[loop] == IOCONFIG_SERVO)
          {
            flag=1;
            _temp -= ServoTimer[loop];
          }

  //  ServoTimer[3]=_temp;
    if(flag)
    {
       if(TMR1ON==0)
       {
         //need to turn timer on
           ServoIndex=0;

           TMR1 = (~_temp)+1;
#if _XTAL_FREQ == 16000000
           T1CON = 0b00100000;  // fsoc/4/4  (1Mhz timer clock)
#else
           // assume 32Mhz clock
           T1CON = 0b00110000;  // fsoc/4/8  (1Mhz timer clock)
#endif
           TMR1GE=0;
           TMR1IF=0;
           TMR1IE=1;
           TMR1ON=1;

       }


    }
    else
    {
        TMR1ON=0;
        TMR1IF=0;
        TMR1IE=0;
    }

}