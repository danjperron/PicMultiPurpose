#ifndef IOCONFIG
#define IOCONFIG

//  IOCONFIG
// ATTENTION IOCONFIG_ANALOG...  VALUE ARE DIRECTLY RELATED TO FVRCON.ADFVR
#define IOCONFIG_ANALOGVDD 0  // don't change
#define IOCONFIG_ANALOG1V  1  //   "     "
#define IOCONFIG_ANALOG2V  2  //   "     "
#define IOCONFIG_ANALOG4V  3  //   "     "
#define IOCONFIG_INPUT     4
#define IOCONFIG_INPUT_PULLUP 5
#define IOCONFIG_OUTPUT    6

#define IOCONFIG_CAP_SENSE_OFF    8
#define IOCONFIG_CAP_SENSE_LOW 	  9
#define IOCONFIG_CAP_SENSE_MEDIUM 10
#define IOCONFIG_CAP_SENSE_HIGH   11


#define IOCONFIG_DHT11     16
#define IOCONFIG_DHT22     17
#define IOCONFIG_DS18B20   32
#define IOCONFIG_SERVO     64
#define IOCONFIG_COUNTER   128








// if we use a RS485 DRIVER without the need to switch
// direction, we wil have an other input

#define INPUT_COUNT 2

#define IO0 RA0
#define IO0_TRIS  TRISAbits.TRISA0
#define IO0_ANSEL ANSELAbits.ANSA0
#define IO0_AN_CHANNEL 0
//#define IO0_IOCAN IOCANbits.IOCAN0
#define IO0_PULLUP WPUAbits.WPUA0

#define IO1 RA1
#define IO1_TRIS  TRISAbits.TRISA1
#define IO1_ANSEL ANSELAbits.ANSA1
#define IO1_AN_CHANNEL 1
//#define IO1_IOCAN IOCANbits.IOCAN1
#define IO1_PULLUP WPUAbits.WPUA1

typedef struct{
unsigned char SlaveAddress;
unsigned char IOConfig[INPUT_COUNT];
}SettingStruct;


extern SettingStruct Setting;

//#define BAUD 9600
//#define BAUD 115200
#define BAUD 57600

#ifndef _XTAL_FREQ
 // Unless specified elsewhere, 4MHz system frequency is assumed
//#define _XTAL_FREQ 16000000
#define _XTAL_FREQ 32000000
#endif

#endif