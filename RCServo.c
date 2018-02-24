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

unsigned int ServoTimerDeadTime=SERVO_REFRESCH_PERIOD;

void RCServoISR(void) {
    
    TMR1ON = 0;
    TMR1IF = 0;
    
    if (ServoTimeOutFlag) {
        if (ServoIndex == 0)
            IO0 = 0;
        else if (ServoIndex == 1)
            IO1 = 0;
    }
    
    do {
        ServoIndex++;
        if (ServoIndex > INPUT_COUNT) {
            ServoIndex = 0;
            ServoTimerDeadTime=SERVO_REFRESCH_PERIOD;
        }
    } while ((Setting.IOConfig[ServoIndex] != IOCONFIG_SERVO) && (ServoIndex<INPUT_COUNT));
    
    if (ServoIndex<INPUT_COUNT) ServoTimerDeadTime -= ServoTimer[ServoIndex];

    if (ServoIndex == INPUT_COUNT)
        CurrentServoTimer = ServoTimerDeadTime;
//          CurrentServoTimer = 16500;
    else        
        CurrentServoTimer = ServoTimer[ServoIndex];
    
    if (CurrentServoTimer == 0) {
        ServoTimeOutFlag = 0;
        TMR1 = 64536;
    } else {
        ServoTimeOutFlag = 1;
        TMR1 = (~CurrentServoTimer) + 1;
    }
    
    TMR1ON = 1;
    if (ServoIndex == 0)
        IO0 = 1;
    else if (ServoIndex == 1)
        IO1 = 1;
}

void DoRCServo(void) {
    unsigned char loop;
    unsigned char flag = 0;
    unsigned short _temp_time, _temp_index;
    
    for (loop = 0; loop < INPUT_COUNT; loop++)
        if (Setting.IOConfig[loop] == IOCONFIG_SERVO) {
            
            //Normalize on periods
            if (ServoTimer[loop] < SERVO_MIN_ACTIVE_TIME) {
                ServoTimer[loop] = SERVO_MIN_ACTIVE_TIME;
            }
            if (ServoTimer[loop] > SERVO_MAX_ACTIVE_TIME) {
                ServoTimer[loop] = SERVO_MAX_ACTIVE_TIME;
            } 
            flag++;
            if (flag==1) {
                _temp_time = ServoTimer[loop];
                _temp_index = loop;
            }
        }

    if (flag) {
        if (TMR1ON == 0) {
            //need to turn timer on
            ServoIndex = _temp_index;
            ServoTimerDeadTime=SERVO_REFRESCH_PERIOD - _temp_index;
            
            TMR1 = (~_temp_time) + 1;
#if _XTAL_FREQ == 16000000
            T1CON = 0b00100000; // fsoc/4/4  (1Mhz timer clock)
#else
            // assume 32Mhz clock
            T1CON = 0b00110000; // fsoc/4/8  (1Mhz timer clock)
#endif
            TMR1GE = 0;
            TMR1IF = 0;
            TMR1IE = 1;
            TMR1ON = 1;

        }


    } else {
        TMR1ON = 0;
        TMR1IF = 0;
        TMR1IE = 0;
    }

}