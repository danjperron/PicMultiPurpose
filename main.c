/* 
 * File:   main.c
 * Author: daniel
 *
 * Created on 27 mars 2014, 21:59
 */

#ifdef __XC__
#include <xc.h>
#else
#include <htc.h>
//#include <stddef.h>
#endif

#include "IOCycle.h"
#include "DHT22.h"
#include "DS18B20.h"
#include "CAPSENSE.h"
#include "IOConfig.h"
#include "RCServo.h"
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Module multi-fonction
//
// Simple programme  pour accéder à deux entrées/sorties qui peuvent être
// configurer.
//
// Les modes suivants sont disponible
//
//
// 1- Entrée digitale.
// 2- Sortie digitale.
// 3- Entrée Analogique avec 1.024,2.048,4.096V et VDD qui voltage de référence.
// 4- Sortie PWM pour R/C servo
// 5- Entrée digital pour lecture capteur AM2302,DHT11 et DHT22.
// 6- Entrée digital pour lecture capteur DS18B20.
// 7- Entrée à effet capacitif avec 3 niveaux de puissance (Pulse/0.1sec).
// 8- Compteur de pulsation (accumulateur 32bits et (Pulse par secondec).

//   Date: 27 Mars 2014
//   programmer: Daniel Perron
//   Version: 1.0
//   Processeur: PIC12F1840
//   logiciel de compilation:  Microchip MPLAB  X IDE (freeware version)
//
//  Le mode de communication est série et le protocol est modbus.
//  
//  Le port RA2 est utilisé pour interfacer unne puce RS-485 genre LTC485.
//  Il est toutefois possible d'utiliser plusieur puce en parallèle directement
//  puisque la sortie RA4 (TX) est en mode entrée lorsque le système
//  ne transmet pas.
//
//
//
//  Registre et mode disponible
//
//
/////  Modbus Fonction 01 & 02
//
//  Lire IO0 et IO1
//
//  Modbus [SLA][1][0][A][0][1][CRC]
//
//  SLA - Adresse
//  A   - 0=IO0 1=IO1
//  CRC - Cyclical redundancy check.
//
//  Example Python avec module minimal modbus
//  IO0 = 0
//  module.read_bit( IO0)
//
//
/////  Modbus Fonction  3
//
//  Modbus [SLA][3][0][A][0][1][CRC]
//
//  A =
//       0 :  Lire valeur R/C servo sur IO0
//       1 :  Lire value  R/C servo sur IO1
//     160 :  Lire Address esclave modbus dur module
//     250 :  Lire Version du logiciel
//     251 :  Lire numéro d'identification software
// (0x100) 256:  Lire Configuration IO0
// (0x101) 257:  Lire Configuration IO1
//
//
// Exemple Python
//   Lire numéro d'identification du logiciel
//
//   id - module.read(251,0,3)
//
//
///// Modbus Fonction 4
//
//  Modbus [SLA][4][0][A][0][Number of register][crc]
//
//  A =  0: Lire information sur IO0
//          Entrée IO0
//          Valeur analogue IO0
//          Capteur et compteur pour IO0  avec plus d'un registre
//
//       1: Lire informatio sur IO1
//          Entrée IO1
//          Valeur analogue IO1
//
//
//      16: Capteur et compteur pour IO1  avec plus d'un registre
//
//      32: Lire Tension de réference 2.048V avec VDD comme référence
//
//      33: Lire Diode de température
//          Une diode seulement avec VDD comme référence
//
///// Modbus Fonction 5
//
//  Modbus [SLA][5][0][A][0][DATA][crc]
//
//  A = 0:  Sortie IO0
//      1:  Sortie IO1
//
//
///// Modbus Fonction 6
//
//  Modbus [SLA][6][0][A][16 bits DATA][CRC]
//
//  A = 0:  Set R/C Servo sur IO0  ou reset compteur
//      1:  Set R/C Servo sur IO1  ou reset compteur
//    160:  Set Adresse modbus du module
// 256(0x100): Set IO0 configuration  (IOCONFIG.H)
// 157(0x101): set IO1 configuration  (IOCONFIG.H)
//
//  exemple python
//  changer IO0 pour lecture capteur DHT22
//
//  module.write_register(0,17,0,6)
//


#define SOFTWARE_ID      0x6532
#define RELEASE_VERSION 0x0100

///////////////////  How to program the IC.
//
//    1 - Use MPLAB with pickit 3  (need version 8.90 at least)
//  or
//    2 - Use standalone pickit2 v 2.61 (Select the I.C. and load the corresponding hex file).
//  or
//    3 - Use Raspberry Pi board  with burnVLP.py . Software available from https://github.com/danjperron/burnLVP
//
////////////////////////////////////  GPL LICENSE ///////////////////////////////////


/*
 This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/



/*  COMMUNICATION PROTOCOL

     baud rate 57600 , 8 bits data , no parity
     just send frequency follow by a cariage return

      using TTL UART with fix baud at 57600, 8 bits data, no parity.


//////////////////////////  PORT DESCRIPTION
/*
 * RA0  IO0     I/O 0
 * RA1  IO1     I/O 1
 * RA2  IO2     RS-485 DIRECTION
 * RA3  IN      MCLR  Master reset
 * RA4  OUT     TX    Serial OUT
 * RA5  IN      RX    Serial IN
*/



//rs-485 data direction

#define TXM_ENABLE   RA2



#ifndef BORV_LO
#define BORV_LO BORV_19
#endif

#define iabs(A)  (A<0 ?  (-A) :  A)

#ifdef __XC__
// CONFIG1
#pragma config FOSC = INTOSC // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF // Flash Memory Self-Write Protection (Write protection off)
   
   
#pragma config PLLEN = OFF // PLL Enable (4x PLL enabled)


#pragma config STVREN = ON // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = ON // Low-Voltage Programming Enable (Low-voltage programming enabled)

#else

__CONFIG(FOSC_INTOSC & WDTE_OFF & PWRTE_OFF & MCLRE_ON & BOREN_OFF & CP_OFF & CPD_OFF & CLKOUTEN_OFF & IESO_OFF & FCMEN_OFF);
__CONFIG(WRT_OFF & PLLEN_ON & BORV_LO & LVP_ON);
__IDLOC(0000);


#endif


//Set default value
//   MODBUS Address, IO0 config , IO1 config
__EEPROM_DATA(127,IOCONFIG_INPUT,IOCONFIG_INPUT,0xff,0xff,0xff,0xff,0xff);



unsigned char VRange;  //1 = 1.024V,  2 = 2.048V, 3 = 4.096V else = VDD

unsigned char BadIO; 
SettingStruct Setting;

#define TIMER_100MS  100
near volatile unsigned short Timerms;       //  Interrupt Timer counter in 1 ms
near volatile unsigned short PrimaryTimerms;
near volatile unsigned char TimerDeciSec;    // modbus timer out in 1/10 of sec  in 0.5 ms count

#pragma pack 1
typedef union {
  unsigned short USHORT;
  unsigned char BYTE[2];
}ByteShortUnion;


near unsigned char CurrentTimer1H;
near unsigned char CurrentTimer1L;

// serial buffer
#define SERIAL_BUFFER_SIZE 32
near volatile unsigned char InFiFo, OutFiFo;   // these are the buffer pointers for interrupt serial communication; InFiFo ctrl by putch, OutFiFo ctrl by interrupt
near volatile unsigned char RcvInFiFo, RcvOutFiFo;
char SerialBuffer[SERIAL_BUFFER_SIZE];
char RcvSerialBuffer[SERIAL_BUFFER_SIZE];

unsigned char SerialSum;    // use for check sum
unsigned char RcvSerialSum;  // use for check sum verification
bit ModbusOnTransmit;



// MODBUS


// CRC16 source code is in CRC16.c file
extern unsigned short CRC16(unsigned char * puchMsg, unsigned char usDataLen);


unsigned char ModbusFunction;
unsigned char ModbusSlave;
unsigned short ModbusAddress;
unsigned short ModbusData;
volatile unsigned short ModbusCRC;
unsigned char ModbusFramePointer;

unsigned char ModbusBuffer[10];

//MODBUS EXCEPTION
#define ILLEGAL_FUNCTION     1
#define ILLEGAL_DATA_ADDRESS 2
#define ILLEGAL_DATA_VALUE   3
#define SLAVE_DEVICE_FAILURE 4
#define ACKNOWLEDGE          5
#define SLAVE_DEVICE_BUZY    6
#define NEGATIVE_AKNOWLEDGE  7
#define MEMORY_PARITY_ERROR  8


/* Timer utilisation
Timer0  500us    interrupt timer
Timer1  Sensor timer  utility, 
Timer2  Servo   Utility
*/





  


// EEPROM LOAD AND SAVE SETTING

void LoadSetting(void)
{
  unsigned char  idx;
  unsigned char  * pointer = (unsigned char *) &Setting;

  for(idx=0; idx < sizeof(Setting);idx++)
     *(pointer++) = eeprom_read(idx);
}

void SaveSetting(void)
{
  unsigned char  idx;
  unsigned char  * pointer = (unsigned char *) &Setting;

  for(idx=0; idx < sizeof(Setting);idx++)
      eeprom_write(idx, *(pointer++));
}



void Init1msTimer()
{
// TIMER2
// 16Mhz clock / 4 =  250 ns clock
// 250ns * 8  = 2us clock
//  1000us / 2us = 250


#if _XTAL_FREQ == 16000000
   T2CON= 0b00000110;
   PR2 = 250;
#else
     // assume 32Mhz
   T2CON= 0b00000111;
   PR2=125;
#endif


 TMR2=0;
 // Enable IRQ
TMR2IF=0;
PrimaryTimerms=100;
TimerDeciSec=10;
TMR2IE=1;
PEIE = 1;
GIE=1;
COUNTER0=0;
COUNTER1=0;
}

void InitA2D()
{
//ADCON1= 0b10100011;  // right justified, fosc/32 vref internal fixed.
ADCON1 =  0b11100011;  // fosc/64  32Mhz
ADCON0= 0b00000001; // enable a/d
ANSELA = 0b0000000;      // NO ANALOG
ADIE=0;
ADIF=0;
FVRCON=0b11000010;  // Vref internal 2.048V on ADC
}

void SetAnalogConfig(unsigned char Pin)
{
// SET ANALOG PIN
 
 unsigned char ioconfig = Setting.IOConfig[Pin];

 if(Pin ==0)
   {
     IO0_TRIS=1;
     IO0_ANSEL=1;
   }
 else
   {
     IO1_TRIS=1;
     IO1_ANSEL=1;
   }

// SET REFERENCE VOLTAGE
   
  FVRCONbits.ADFVR = ioconfig;
 
// UTILISER VREF OU VDD
  if(ioconfig== IOCONFIG_ANALOGVDD)
    ADCON1bits.ADPREF=0;
  else
    ADCON1bits.ADPREF=3;
}

void SetOutputConfig(unsigned char Pin)
{
  if(Pin ==0)
   {
     IO0=0;
     IO0_TRIS=0;
     IO0_ANSEL=0;
   }
  else
   {
     IO1=0;
     IO1_TRIS=0;
     IO1_ANSEL=0;
  }
}




void SetPullUp(unsigned char Pin, unsigned char PullUp)
{
 if(Pin==0)
    IO0_PULLUP= PullUp;
  else
     IO1_PULLUP=PullUp;
}

void SetInputConfig(unsigned char Pin)
{
  if(Pin==0)
   {
      IO0_TRIS=1;
      IO0_ANSEL=0;
   }
  else
   {

     IO1_TRIS=1;
     IO1_ANSEL=0;
   }
}



void SetIOConfig(unsigned char Pin)
{
    unsigned char loop;
  unsigned char ioconfig = Setting.IOConfig[Pin];

  ResetIOCycle();

  IOSensorData[Pin].DWORD=0;
  IOSensorData[Pin].WORD[2]=0;


  if(Pin==0)
      IO0CounterFlag=0;
  if(Pin==1)
      IO1CounterFlag=0;


  SetPullUp(Pin,1);  // By default pull up is there
  SetIOChange(Pin,0);
  ServoTimer[Pin]=0; // disable Servo

  switch(ioconfig) {

    case IOCONFIG_ANALOG1V:
    case IOCONFIG_ANALOG2V:
    case IOCONFIG_ANALOGVDD:
    case IOCONFIG_ANALOG4V:
                            SetPullUp(Pin,0);
                            SetAnalogConfig(Pin);
                            break;
    case IOCONFIG_SERVO:
    case IOCONFIG_OUTPUT:
                            SetOutputConfig(Pin);
                            break;
    case IOCONFIG_COUNTER:
                            SetPullUp(Pin,0);
                            SetInputConfig(Pin);
                            SetIOChange(Pin,1);
                            if(Pin==0)
                                IO0CounterFlag=1;
                            if(Pin==1)
                                IO1CounterFlag=1;

                            break;

    case IOCONFIG_CAP_SENSE_OFF:
    case IOCONFIG_CAP_SENSE_LOW:
    case IOCONFIG_CAP_SENSE_MEDIUM:
    case IOCONFIG_CAP_SENSE_HIGH:
    case IOCONFIG_INPUT:
                            SetPullUp(Pin,0);
                            SetInputConfig(Pin);
                            break;
    default:                SetInputConfig(Pin);
}
}






unsigned short ReadA2D(unsigned char channel)
{

 ByteShortUnion value;
 ADIE=0;                              // clear interrupt flag
 ADIF=0;
 ADON=1;
 ADCON0bits.ADON=1;
 ADCON0bits.CHS=channel;
 __delay_ms(1);
 ADCON0bits.ADGO=1;
 while(ADCON0bits.ADGO==1);
 value.BYTE[1]=ADRESH;
 value.BYTE[0]=ADRESL;
 return value.USHORT;
}


static void interrupt isr(void){
    static volatile unsigned char _temp;
//timer1  rcservo

if(TMR1IE)
  if(TMR1IF)
     RCServoISR();


// check interrupts on change
if(IOCIE)
if(IOCIF)
 {
//    IOCAF=0;
     _TMR0= TMR0;
  TMR0=0;
      _temp = IOCAF & IOCAN;

    if(_temp&1)
    {
        //IO0
        IOCAFbits.IOCAF0=0;

        if(IO0CounterFlag)
        {
            //IOSensorData[0].DWORD++;
            // COUNTER0++;
            // use assembly since variable need to be   big endian
            // MODBUS use BIG ENDIAN

#asm
        movlw 1
        banksel(_IOSensorData)
        addwf ((_IOSensorData+3)^128),f
        movlw 0
        addwfc ((_IOSensorData+2)^128),f
        addwfc ((_IOSensorData+1)^128),f
        addwfc (_IOSensorData^128),f
        movlw 1
        banksel(_COUNTER0)
        addwf ((_COUNTER0+1)^128),f
        movlw 0
        addwfc (_COUNTER0^128),f

#endasm
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
          //  IOSensorData[1].DWORD++;
          //  COUNTER1++;
          // use assembly since variable need to be   big endian

#asm
        movlw 1
        banksel(_IOSensorData)
        addwf ((_IOSensorData+9)^128),f
        movlw 0
        addwfc ((_IOSensorData+8)^128),f
        addwfc ((_IOSensorData+7)^128),f
        addwfc ((_IOSensorData+6)^128),f
        movlw 1
        banksel(_COUNTER1)
        addwf ((_COUNTER1+1)^128),f
        movlw 0
        addwfc (_COUNTER1^128),f

#endasm


        }
        else if(CurrentIOSensor & IOCONFIG_DHT11)
            DHT22IOCAF();
    }

//  DealWithIOCAF();

 }

// Timer 1 ms
if(TMR2IF){
 TMR2IF=0;
 if(TimerSecFlag)
 {


    if(IO0CounterFlag)
     {
         IOSensorData[0].WORD[2]=COUNTER0;
         COUNTER0=0;
     }
     if(IO1CounterFlag)
     {
         IOSensorData[1].WORD[2]=COUNTER1;
         COUNTER1=0;
     }

     TimerSecFlag=0;
 }

 Timerms++;
 PrimaryTimerms--;
 if(PrimaryTimerms==0)
  {
     PrimaryTimerms=TIMER_100MS;
    if(WaitForEndDeciSecond)
    {
        _TMR0=TMR0;
        CPSON=0;
        TMR0IE=0;
        WaitForEndDeciSecond=0;
        GotCapSenseFlag=1;
       
    }
    else if(WaitForStartDeciSecond)
    {
        WaitForStartDeciSecond=0;        
        TMR0=0;
        TMR0IF=0;
        TMR0IE=1;
        WaitForEndDeciSecond=1;
       
    }
     TimerDeciSec--;
     if(TimerDeciSec==0)
     {
         TimerDeciSec=10;
         TimerSecFlag=1;
     }


  }
}
// check serial transmit Interrupt
if(TXIE)
 if(TXIF)
  {
     // do we have a new char to send
    if(InFiFo != OutFiFo)
      {
        TXREG= SerialBuffer[OutFiFo];
        OutFiFo++;
       if(OutFiFo >= SERIAL_BUFFER_SIZE)
         OutFiFo=0;
      }
     else
   if(OutFiFo == InFiFo)
     {
       // nothing in buffer  disable tx interrupt
       TXIE=0;
     }
  }

// check serial  receive
if(RCIE)
 if(RCIF)
   {
     RcvSerialBuffer[RcvInFiFo++]= RCREG;
 //    RCREG = RcvSerialBuffer[RcvInFiFo++]
     if(RcvInFiFo == SERIAL_BUFFER_SIZE)
        RcvInFiFo=0;
   }


  if(TMR0IE)
  if(TMR0IF)
   {
      TMR0IF=0;
      if(Timer0Overflow)
      {
          _TMR0_MSB++;
      }
      else
      {
     // got timer0 time out
    TMR0IE=0;
    CurrentIOStatus=IO_STATUS_BAD;
    CurrentIOCycle=IO_CYCLE_END;
      }
  }

}




void putch(char char_out)
{
   unsigned char temp;

    SerialSum+= (unsigned char) char_out;
// increment circular pointer InFiFo
   temp = InFiFo + 1;
   if(temp >= SERIAL_BUFFER_SIZE)
     temp = 0;

//  wait  if buffer full
  while(temp == OutFiFo);

// ok write the buffer
  SerialBuffer[InFiFo]=char_out;
// now tell the interrupt routine we have a new char
InFiFo= temp;

// and enable interrupt
 TXIE=1;
}




unsigned char  RcvGetBufferLength(void)
{
   unsigned char temp;

   temp =  SERIAL_BUFFER_SIZE;
   temp += RcvInFiFo;
   temp -= RcvOutFiFo;

   return (temp % SERIAL_BUFFER_SIZE);
}

void RcvClear(void)
{
    GIE=0;
    RcvInFiFo=0;
    RcvOutFiFo=0;
    GIE=1;
}
unsigned char RcvIsDataIn(void)
 {
     return (RcvInFiFo == RcvOutFiFo ? 0 : 1);
 }

char RcvGetChar(void)
 {
    char temp;

   // wait until we received something
   while(!RcvIsDataIn());

  // get the character
    temp =  RcvSerialBuffer[RcvOutFiFo];
    RcvOutFiFo++;
    if(RcvOutFiFo >= SERIAL_BUFFER_SIZE)
      RcvOutFiFo=0;

    return temp;
}


void TXM_WAIT(void)
{
    while(TXIE);
    __delay_us(200);
    TXM_ENABLE=0;
}

void SendModbusPacket(char * buffer, int BufferSize)
{
    unsigned short CRC;
    unsigned char loop;
    CRC = CRC16(buffer,BufferSize);
    //RS-485 on TRANSMISSION
    ModbusOnTransmit=1;
    TXM_ENABLE=1;
    // send data
    for(loop=0;loop<BufferSize;loop++)
        putch(buffer[loop]);
    // send CRC
    putch(CRC & 0xFF);
    putch(CRC >> 8);

    //ModbusOnTransmit=0;
    // wait until no more to transmit
    //while(TXIE);

    //_delay_us(200);

    // RS-485 on RECEPTION
    //TXM_ENABLE=0;

}

void  SendFrameError(unsigned char Function , unsigned char ErrorCode)
{
    unsigned char buffer[3];

    buffer[0]= Setting.SlaveAddress;
    buffer[1]= Function | 0x80;
    buffer[2]= ErrorCode;
    SendModbusPacket(buffer,3);
}




void SendReadByteFrame(unsigned  char value)
{
   unsigned char buffer[4];

   buffer[0]= Setting.SlaveAddress;
   buffer[1]= ModbusFunction;
   buffer[2]= 1;  // byte count
   buffer[3]= value;

   SendModbusPacket(buffer,4);
 }

void SendReadFrame(unsigned  short value)
{
   unsigned char buffer[5];

   buffer[0]= Setting.SlaveAddress;
   buffer[1]= ModbusFunction;
   buffer[2]= 2;  // byte count
   buffer[3]= value >> 8;
   buffer[4]= value & 0xff;

   SendModbusPacket(buffer,5);
}


void SendBytesFrame(unsigned char _Address)
{
  unsigned char loop;
  unsigned char NByte;
  unsigned char buffer[10];
  unsigned char _temp;

  unsigned char SensorId;
  if(_Address==0)
      SensorId=0;
  else
      SensorId=1;
  switch(Setting.IOConfig[SensorId])
   {

      case IOCONFIG_DHT22:
      case IOCONFIG_DHT11: 
      case IOCONFIG_DS18B20: 
      case IOCONFIG_COUNTER: NByte = 6 ; break;
      case IOCONFIG_CAP_SENSE_OFF:
      case IOCONFIG_CAP_SENSE_LOW:
      case IOCONFIG_CAP_SENSE_MEDIUM:
      case IOCONFIG_CAP_SENSE_HIGH: NByte=4; break;
      default:  NByte=0;
   }

    if(NByte==0)
         SendFrameError(ModbusFunction | 0x80, ILLEGAL_DATA_ADDRESS);
    else
     {
       buffer[0]= Setting.SlaveAddress;
       buffer[1]= ModbusFunction;
       buffer[2]= NByte;


       for(loop=0;loop<NByte;loop++)
        {
           _temp= IOSensorData[SensorId].BYTE[loop];
          buffer[3+loop]=_temp;
        }

       SendModbusPacket(buffer,NByte+3);
     }   
}


void SendPresetFrame()
{
   unsigned char buffer[6];

   buffer[0]= Setting.SlaveAddress;
   buffer[1]= ModbusFunction;
   //buffer[2]= 2;  // byte count

   buffer[2]= ModbusAddress >> 8;
   buffer[3]= ModbusAddress & 0xff;

   buffer[4]= ModbusData >> 8;
   buffer[5]= ModbusData & 0xff;


   SendModbusPacket(buffer,6);
   
}



unsigned char DecodeSerial(char * msg)
{
    int loop;
    unsigned char rcode;
    unsigned short CalcCRC;

    unsigned char * pt=msg;

    ModbusSlave= *(pt++);
    ModbusFunction= *(pt++);


    #define ToUSHORT    ((((unsigned short)*pt) * 256) + ((unsigned short) pt[1]));pt+=2;

    ModbusAddress= ToUSHORT;
    ModbusData= ToUSHORT;

    // MODBUS CRC have LSB FIRST
    ModbusCRC= *pt++;
    ModbusCRC|= ((unsigned short)(*pt)) << 8;


    CalcCRC = CRC16(ModbusBuffer,6);

   if(CalcCRC != ModbusCRC) rcode=0;
   else if(ModbusSlave==Setting.SlaveAddress) rcode=1;
   else rcode=2;

   return rcode;

}




  // Function 3  Read Holding Register
  //
  // Address 0: Read R/C servo on IO0 if enable
  // Address 1: Read R/C servi on IO1 if enable
  // Address 160: SlaveAddress (Node ID)
  // Address 0x100: Read IO0 Config
  // Address 0x101: Read IO1 Config
  // Address 250: Version Number
  // Address 251: Software ID NUMBER

void   ReadHoldingRegister()
{
    unsigned short temp;
    char Flag= 0;

   if(ModbusAddress == 0x100)
      temp = Setting.IOConfig[0];
   else if(ModbusAddress == 0x101)
      temp = Setting.IOConfig[1];
   else if(ModbusAddress == 160)
      temp = Setting.SlaveAddress;
   else if(ModbusAddress == 250)
      temp = RELEASE_VERSION;
   else if(ModbusAddress == 251)
      temp = SOFTWARE_ID;
   else if(ModbusAddress == 0)
      temp= ServoTimer[0];
   else if(ModbusAddress == 1)
      temp= ServoTimer[1];
   else
      Flag= 1;

    if(Flag)
     SendFrameError(ModbusFunction | 0x80 , ILLEGAL_DATA_ADDRESS);
    else
     SendReadFrame(temp);
}


unsigned short ReadIO(unsigned char Pin)
{
  
  BadIO=0;  // clean Bad IO 
  unsigned char ioconfig = Setting.IOConfig[Pin];
  unsigned short temp;
  // ANALOG MODE
  if(ioconfig <= IOCONFIG_ANALOG4V)
    {
      SetAnalogConfig(Pin);  // set the analog VRef
      if(Pin==0)
          return ReadA2D(IO0_AN_CHANNEL);
      else
        return ReadA2D(IO1_AN_CHANNEL);
    } 

  // INPUT  & OUTPUT  MODE
  if(ioconfig <= IOCONFIG_OUTPUT)
  {
    if(Pin==0)
      return IO0;
    else
      return IO1;
  }
  else
      return(0xffff);
  
  // I/O METHODE A COMPLETER
  // IOCONFIG_DHT11 
  // IOCONFIG_DHT22     
  // IOCONFIG_DS18B20   
  // IOCONFIG_PWM       
  // IOCONFIF_SERVO     

  // A COMPLETER
//  BadIO=1; // Function non valide 
}

unsigned short ReadVRef()
{
  // A/D INPUT = FVR (2.048V)
  FVRCONbits.ADFVR=2;
  // A/D VREF = VDD
  ADCON1bits.ADPREF=0;
  return ReadA2D(31);
}

unsigned short ReadTSensor()
{
  // A/D VREF = VDD
  FVRCONbits.TSEN=1;
  FVRCONbits.TSRNG=0;
  ADCON1bits.ADPREF=0;
  return ReadA2D(29);
}


unsigned char  MultipleRegister(unsigned char _Address)
{
    if(_Address == 0)
        if(Setting.IOConfig[0] & (IOCONFIG_CAP_SENSE_OFF | IOCONFIG_DHT11 | IOCONFIG_DS18B20 | IOCONFIG_COUNTER))
            return 1;
    if(_Address == 0x10)
        if(Setting.IOConfig[1] & (IOCONFIG_CAP_SENSE_OFF | IOCONFIG_DHT11 | IOCONFIG_DS18B20 | IOCONFIG_COUNTER))
            return 1;
        return 0;
}


  // Function 4  Read Current Register
  //
  // Address 0:  Current IO sensor 0
  // Address 1:  Current IO sensor 1
  // Address 32:  Current VRef 2.048V A/D value
  // Address 33:  Current Build-in Temperature Sensor

void   ReadCurrentRegister()
{
    unsigned short temp;

    // only the first 1 address possible
    // and only 1 address at a time


 //   puts("ModbusAddress=");
 //   printHexUShort(ModbusAddress);
 //   puts("\r\nModbusData=");
 //   printHexUShort(ModbusData);
 //   puts("\r\n");
   

  // if(ModbusData != 1)
  //     SendFrameError(ModbusFunction | 0x80, ILLEGAL_DATA_ADDRESS);
 //  else
   {
       temp=0;
       BadIO=0;
       if(MultipleRegister(ModbusAddress))
       {
           SendBytesFrame(ModbusAddress);
           return;
       }
       switch(ModbusAddress)
       {
           case 0: temp = ReadIO(0);break;  // Read IO0
           case 16: temp = ReadIO(1);break;  // Read IO1
           case 32: temp = ReadVRef();break; // Read 2.048V reference Value
           case 33: temp = ReadTSensor();break; // Read Build-in Temperature sensor
           default: BadIO=1;
       }
       if(BadIO)
         SendFrameError(ModbusFunction | 0x80, ILLEGAL_DATA_ADDRESS);
       else
         SendReadFrame(temp);
   }
}


void ReadInputStatus()
{
  // Read Coil ou Read Input Status
  // seulement starting address 0 et 2 points possible
   if(ModbusAddress == 0)
      SendReadByteFrame(IO0);
   else if(ModbusAddress ==1)
      SendReadByteFrame(IO1);
   else
      SendFrameError(ModbusFunction | 0x80 , ILLEGAL_DATA_ADDRESS);  
}

void ForceSingleCoil()
{
    if(ModbusAddress < INPUT_COUNT)
    {
       if(Setting.IOConfig[ModbusAddress] == IOCONFIG_OUTPUT)
         {
           if(ModbusAddress==0)
             {
              IO0_TRIS=0;
              IO0_ANSEL=0;
              IO0 = ModbusData > 0 ? 1 : 0;
             }
           else
             {
              IO1_TRIS=0;
              IO1_ANSEL=0;
              IO1 = ModbusData > 0 ? 1 : 0;
             }
           SendPresetFrame();
           return;     
         }
    }
    SendFrameError(ModbusFunction | 0x80 , ILLEGAL_DATA_ADDRESS);
}


  // Function 6 Preset Single Register
  //
  // Address 0x100: IOConfig I0 0
  // Address 0x101: IOConfig IO 1
  // Address 0x0: Set RCServo0 and clear counter accumulator
  // Address 0x1: Set RCServo1 and clear counter accumulator
  // Address 160: Slave Address

void PresetSingleRegister()
{
  unsigned char oldConfig;
  unsigned char temp;
    if((ModbusAddress ==0x100) || (ModbusAddress == 0x101))
    {
      temp = ModbusAddress - 0x100;
      BadIO=0;
      oldConfig=Setting.IOConfig[temp];
      Setting.IOConfig[temp]=ModbusData;
      SetIOConfig(temp);
      if(BadIO)
        {
         Setting.IOConfig[temp]=oldConfig;
         SetIOConfig(temp);
         SendFrameError(ModbusFunction | 0x80 , ILLEGAL_DATA_ADDRESS);
        }
      else
        {
          SendPresetFrame();
          SaveSetting();
        }
    }
    else if(ModbusAddress == 0)
    {
        if(Setting.IOConfig[0]==IOCONFIG_COUNTER)
        {
            if(ModbusData==0)
                IOSensorData[0].DWORD=0;
        }
        else
         ServoTimer[0]=ModbusData;
        SendPresetFrame();
    }
    else if(ModbusAddress == 1)
    {
         if(Setting.IOConfig[1]==IOCONFIG_COUNTER)
        {
            if(ModbusData==0)
                IOSensorData[1].DWORD=0;
        }
        else
          ServoTimer[1]=ModbusData;
          SendPresetFrame();
    }

    else if(ModbusAddress == 160)
    {
      Setting.SlaveAddress=ModbusData;
      SaveSetting();
      SendPresetFrame();
    }
    else
       SendFrameError(ModbusFunction | 0x80 , ILLEGAL_DATA_ADDRESS);
}


void ExecuteCommand(void)
{

  if(ModbusSlave != Setting.SlaveAddress)
     return;    // this is not our Slave Address! just forget it


      __delay_us(100);

 // if(ModbusLRC != ModbusCheckSum)
 //     return; // invalide check sum we should deal with it

  if(ModbusFunction == 1)
      ReadInputStatus();
  else if(ModbusFunction == 2)
      ReadInputStatus();
  else if(ModbusFunction == 3)
      ReadHoldingRegister();
  else if(ModbusFunction == 4)
      ReadCurrentRegister();
  else if(ModbusFunction == 5)
      ForceSingleCoil();  
  else if(ModbusFunction == 6)
      PresetSingleRegister();
  else
     SendFrameError(ModbusFunction | 0x80, ILLEGAL_FUNCTION);
}



 main(void){
     unsigned char loop;
     unsigned char rcode;


#if _XTAL_FREQ == 16000000
 OSCCON		= 0b01111011;	// 16MHz  internal clock
 OPTION_REG	= 0b00000001;	// pullups on, TMR0 @ Fosc/4/4
#else
 // assume 32Mhz
 OSCCON		= 0b11110000;	// 32MHz  internal clock
 OPTION_REG	= 0b00000010;	// pullups on, TMR0 @ Fosc/4/8
#endif
 ANSELA		= 0;	// NO Analog
 PORTA   	= 0b00100000;
 WPUA		= 0b00111111;	// pull-up ON

 


 INTCON		= 0b00000000;	// no interrupt

 // Set Default rn

 LoadSetting();


 TRISA		= 0b00101011;	// RA0,RA1,RA3,RA5 INPUT , RA2,RA4 OUTPUT
 TXM_ENABLE=0;

 // set serial com with 9600 baud
//alternate pin
 APFCON = 0b10000100;
    
 TXSTA = 0b10000010;
 RCSTA = 0;


#if BAUD == 9600
 BRGH =0; //8mhz =>1;
 BRG16 = 1;
 SYNC =0;


#if _XTAL_FREQ == 16000000
 SPBRGL = 103;  //9600  baud
#else
 SPBRGL = 207; // assume 32Mhz clock
#endif
 SPBRGH =0;

#elif BAUD == 115200
 // assume  baud 57600
 BRGH =1;
 BRG16 = 1;
 SYNC =0;


#if _XTAL_FREQ == 16000000
 SPBRGL = 34;  // baud
#else
 SPBRGL = 68; // assume 32Mhz clock
#endif
 SPBRGH =0;

#else

// assume  baud 57600
 BRGH =1;
 BRG16 = 1;
 SYNC =0;


#if _XTAL_FREQ == 16000000
 SPBRGL = 68;  //57600 baud
#else
 SPBRGL = 138; // assume 32Mhz clock
#endif
 SPBRGH =0;

#endif


 TXEN =1;   // enable transmitter
 SPEN = 1;  // enable serial port
 CREN = 1;  // enable receiver
 TXIE =0;   // disable transmit interrupt
 RCIF =0;   // clear received flag
 TXIF = 0;
 SCKP = 0;
 ABDEN = 0;
// reset interrupt fifo buffer
 InFiFo=0;
 OutFiFo=0;
 RcvInFiFo=0;
 RcvOutFiFo=0;


 GIE = 1;
 PEIE =1;   // enable peripheral
 RCIE =1;   // Enable received interrupt
 IOCAP =0;
 IOCAN = 0;
 IOCAF = 0;
 IOCIE = 1; // enable interrupt on change


IO0CounterFlag=0;
IO1CounterFlag=0;
ModbusOnTransmit=0;

 Init1msTimer() ;
 InitA2D() ;

 SetIOConfig(0);
 SetIOConfig(1);
   

    // clear Modbus system first
 RcvClear();
 ModbusFramePointer=0;
 // wait for  serial uart  i/o pin toggle ( at least one full caracter length)
  __delay_ms(5000);
  //cputs("Water Detect V1.01\n\r");

  ResetIOCycle();

  for(loop=0;loop<INPUT_COUNT;loop++)
      ServoTimer[loop]=0;
 

 while(1)
 {

       // No more transmission. put the system back on reception
       // let's
       if(!ModbusOnTransmit)
           TXM_ENABLE=0;
     if(ModbusOnTransmit)
     {
         if(!TXIE)
         {
             __delay_us(200);
             TXM_ENABLE=0;
             ModbusOnTransmit=0;
         }
     }
     else
     if(RcvIsDataIn())
     {

         ModbusBuffer[ModbusFramePointer++]=RcvGetChar();
         if(ModbusFramePointer>=8)
         {
             ModbusFramePointer=8;
             rcode = DecodeSerial(ModbusBuffer);
          if(rcode==1)
          {
              ExecuteCommand();
             ModbusFramePointer=0;
          }
          else if(rcode ==2)
          {
              // ok not this slave
              ModbusFramePointer=0;
          }
          else
          {
              // something wrong then just shift data
              for(loop=1;loop<8;loop++)
                  ModbusBuffer[loop-1]=ModbusBuffer[loop];
              ModbusFramePointer--;
          }
         }
     }

     DoIOCycle();
     DoRCServo();
 }

}





