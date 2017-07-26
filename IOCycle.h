#ifndef IOCYCLE
#define IOCYCLE



#include "IOConfig.h"

#define IO_CYCLE_IDLE          0
#define IO_CYCLE_START         1


#define IO_CYCLE_WAIT          98
#define IO_CYCLE_END           99
extern near volatile unsigned short Timerms;
extern unsigned char CurrentIOSensor;
extern near unsigned char CurrentIOPin;
extern near unsigned char CurrentIOCycle;
extern unsigned char CurrentIOStatus;
extern  bit      Timer0Overflow;
extern  bit      ServoTimeOutFlag;
extern  bit      TimerSecFlag;
extern  bit      IO0CounterFlag;
extern  bit      IO1CounterFlag;

extern unsigned short COUNTER0;
extern unsigned short COUNTER1;


#define IO_STATUS_UNKNOWN 0
#define IO_STATUS_OK      1
#define IO_STATUS_BAD     0xff


#define SENSOR_DATA_MAX 3
#define SENSOR_DATA_BYTE_MAX (SENSOR_DATA_MAX * 2)


typedef union {
    unsigned char BYTE[SENSOR_DATA_BYTE_MAX];
    unsigned short WORD[SENSOR_DATA_MAX];
    unsigned long  DWORD;
}SensorDataUnion; 

extern SensorDataUnion  WorkingSensorData;
extern SensorDataUnion  IOSensorData[INPUT_COUNT];


extern void DoIOCycle(void);
//extern void DealWithIOCAF(void );
extern void ResetIOCycle(void);
extern void SetIOChange(unsigned char Pin, unsigned char value);


extern volatile unsigned char _TMR0;
extern volatile unsigned short _TMR0_MSB;
extern volatile unsigned char BitCount;
extern volatile unsigned char WorkingByte;
extern volatile unsigned char CSum;
extern volatile unsigned char WorkingCount;
extern volatile unsigned  char ByteIndex;
extern volatile unsigned char PreBitCount;


#endif
