/**************************************
PIC Multi-Purpose Modbus

configuration application 

This program is used to set the Modbus Address of the PIC Multi-purpose module

it will also set the IO pin mode configuration

to compile:

   gcc -I /usr/include/modbus configPIC.c -o configPIC  -l modbus

libmodbus is needed

 please check url: http://libmodbus.org
 
 You could  download libmodbus on the Raspberry Pi using

    sudo apt-get install libmodbus5 libmodbus-dev

PIC Multi-Purpose  configPIC  program
Daniel Perron (c)  April, 2014
V1.0
***************************************/

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

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <termios.h>

#include <modbus.h>

typedef struct
{
unsigned short     mode;
unsigned char * asciiMode;
}IOConfigStruct;

char device[256];
int Baud = 57600;

unsigned char scanOnly=0;
unsigned char CurrentSlaveAddress=255;

int IsModuleFound(modbus_t * mb, unsigned char  SlaveAddress);
unsigned char selectModule(modbus_t * mb);


IOConfigStruct  configInfo[]={
  {  0, "ANALOGVDD"},
  {  1, "ANALOG1V"},
  {  2, "ANALOG2V"},
  {  3, "ANALOG4V"},
  {  4, "INPUT"},
  {  5, "INPUT PULLUP"},
  {  6, "OUTPUT"},
  {  8, "CAP SENSE OFF"},
  {  9, "CAP SENSE LOW"},
  { 10, "CAP SENSE MEDIUM"},
  { 11, "CAP SENSE HIGH"},
  { 16, "DHT11"},
  { 17, "DHT22"},
  { 32, "DS18B20"},
  { 64, "R/C SERVO"},
  {128, "COUNTER"},
  {65535, NULL}};


int IsConfigModeValid(int mode)
{
  int loop;

  for(loop=0;;loop++)
   {
     if(configInfo[loop].mode==65535) break;
     if(configInfo[loop].mode==mode) return 1;
   }

 return 0;
}

char * configModeInText(int IOConfigMode)
{
   int loop;

   for(loop=0;;loop++)
    {
      if(configInfo[loop].mode==65535)
         break;
      if(configInfo[loop].mode== IOConfigMode) 
         return configInfo[loop].asciiMode;
    }

   return "???";
}


void decodeArg(int argc, char * argv[])
{
  int loop;

  for(loop=1;loop<argc;loop++)
   {
    if(strcasecmp(argv[loop],"-scan")==0)
      scanOnly=1;
    if(strcmp(argv[loop],"-d")==0)
      {
       loop++;
       strcpy(device,argv[loop]);
      }
    if(strcmp(argv[loop],"-b")==0)
      {
        loop++;
        Baud=atoi(argv[loop]);
      }
    if(strcmp(argv[loop],"--help")==0)
      {
        printf("usage:\nconfigPIC  [-scan][-d DeviceName][-b BaudRate]\n");
        exit(0);
      }
  }
}


void changeAddress(modbus_t * mb)
{

  int  new_Address;

  printf("\n=======Change Address\n");





  if(CurrentSlaveAddress == 255)
     selectModule(mb);
  else
     if(!IsModuleFound(mb,CurrentSlaveAddress))
       selectModule(mb);

  if(CurrentSlaveAddress==255) return;


   printf("\nEnter new Slave Address for this module (1..127) ?");

   if(scanf("%d",&new_Address)!=1)
      printf("Invalid value!  configuration not change.\n");
   else
     {
       if(new_Address>0)
        if(new_Address<128)
        {
          if(IsModuleFound(mb,new_Address))
          {
      	   modbus_set_slave(mb,CurrentSlaveAddress);
           printf("Address already used!\nConfiguration not changes.\n");
          }
          else
          {
   	   modbus_set_slave(mb,CurrentSlaveAddress);
           modbus_write_register(mb,160,new_Address);
           printf("Module is now on Address %d\n",new_Address);
           CurrentSlaveAddress=new_Address;
          }
        }
      else
       printf("Invalid value!  configuration not change.\n");
     }
   usleep(1000000); // delay needed to store data into eerom
   IsModuleFound(mb,CurrentSlaveAddress);

}


unsigned short  getConfigMode(modbus_t * mb, int Pin)
{
     uint16_t MB_Register;
     if( modbus_read_registers(mb,0x100+Pin,1,&MB_Register) < 0)
        return 65535;
     return MB_Register;
}

int IsModuleFound(modbus_t * mb, unsigned char  SlaveAddress)
{
   uint16_t MB_Register;
   int rcode;

   modbus_set_slave(mb,127);

//  while(1)
//   modbus_read_registers(mb,251,1,&MB_Register);
  

   if(SlaveAddress >127) return 0;

   modbus_set_slave(mb,SlaveAddress);
   if (modbus_read_registers(mb,251,1,&MB_Register) <0) return 0;
   if(MB_Register != 0x6532)
     return 0;
   return 1;
}


unsigned char selectModule(modbus_t * mb)
{
   char buffer[256];
   int SlaveAddress;
   int rcode;
   uint16_t MB_Register;
   CurrentSlaveAddress=255;

   printf("Select Module\nEnter Slave Address ?");

   if(scanf("%d",&SlaveAddress)==1)
      if(SlaveAddress>0)
       if(SlaveAddress<128)
         if(IsModuleFound(mb,SlaveAddress))
           CurrentSlaveAddress=SlaveAddress;

  printf("\n");fflush(stdout);
  return SlaveAddress;
}



void changeConfigMode(modbus_t * mb,int Pin)
{
  
  int loop;
  unsigned short  mode;
  unsigned char  module;
  int  new_mode;
  uint16_t MB_mode;

  printf("\n=======  Change  IO%d mode\n",Pin);




  if(CurrentSlaveAddress == 255)
     selectModule(mb);
  else
     if(!IsModuleFound(mb,CurrentSlaveAddress))
       selectModule(mb);

  if(CurrentSlaveAddress==255) return;

  for(loop=0;;loop++)
    {

      if(configInfo[loop].mode==65535) break;

      if(loop%3==0)
        printf("\n");
      printf("%3d) %-20s",configInfo[loop].mode, configInfo[loop].asciiMode);
    }

   mode=getConfigMode(mb,Pin);
   printf("\n\nSlave address %d  current IO%d mode is %d : %-20s\n",CurrentSlaveAddress,Pin,mode,configModeInText(mode));

   printf("\nEnter new configuration ?");

   if(scanf("%d",&new_mode)!=1)
      printf("Invalid value!  configuration not change.\n");
   else
     {
      if(IsConfigModeValid(new_mode))
        {
          MB_mode= new_mode;
          modbus_write_register(mb,0x100+Pin,MB_mode);
         printf("Module %d  IO%d set to %d: %s\n",CurrentSlaveAddress,Pin,MB_mode,configModeInText(MB_mode));
        }
      else
       printf("Invalid value!  configuration not change.\n");
     }
   usleep(1000000); // delay needed to store data into eerom
}

void scanBus(modbus_t * mb)
{
  int loop,io;
  int rcode;
  uint16_t  MB_Register;



//set 1/100th of second response time
   struct timeval response;
   response.tv_sec=0;
   response.tv_usec=10000;
 modbus_set_response_timeout(mb, &response);


  printf("\n==== Scanning MODBUS\n");fflush(stdout);

  for(loop=1;loop<128;loop++)
  {

   if(IsModuleFound(mb,loop))
   {
    printf("%d : %-20s",loop,"PIC Multi-Purpose");

    for(io=0;io<2;io++)
    {
      printf("IO%d: ",io);
      printf("%-20s",configModeInText(getConfigMode(mb,io)));
    }
    printf("\n");
   }
 }
}


unsigned char ReadOneKey(void)
{
  char c;
  struct termios term,term_orig;

  // put keyboard in raw mode

  tcgetattr(0,&term_orig);
  memcpy(&term,&term_orig,sizeof(struct termios));
  term.c_lflag &= ~(ICANON | ECHO);
  term.c_cc[VTIME] =0;
  term.c_cc[VMIN]=1;
  tcsetattr(0,TCSANOW, &term);

  while(read(0, &c,1)!=1);

  tcsetattr(0,TCSANOW,&term_orig);
  fflush(stdin);
  return c;
}



void Menu(modbus_t * mb)
{
char  rcode;

 printf("\nPIC multi-purpose I/O  MODBUS configuration\n");
 printf("Version 1.0 (c) Daniel Perron, April 2014\n");
 printf("device:%s Baud Rate:%d\n",device,Baud);
 while(1)
  {
    // print menu
    printf("\n\n\nM) MODBUS scan \n");
    printf("S) Select Slave Module\n");
    printf("A) Change Slave Address\n");
    printf("0) Set IO0 mode\n");
    printf("1) Set IO1 mode\n");
    printf("Q) Quit\n");


    printf("\nSelected module :");

    if(CurrentSlaveAddress == 255)
     printf("None\n");
    else
     printf("%d\n",CurrentSlaveAddress);

    fflush(stdout);

    rcode= toupper(ReadOneKey());
    fflush(stdin);
    switch(rcode)
    {
     case 'M':  scanBus(mb);break;
     case 'S':  selectModule(mb);break;
     case 'A':  changeAddress(mb);break;
     case '0':  changeConfigMode(mb,0);break;
     case '1':  changeConfigMode(mb,1);break;
     case 'Q':  return;
    }
  }

}






int main(int argc, char * argv[])
{

   modbus_t *mb;

  strcpy(device,"/dev/ttyAMA0");

  decodeArg(argc,argv);

   mb = modbus_new_rtu(device,Baud,'N',8,1);
   modbus_connect(mb);

if(scanOnly==1)
  scanBus(mb);
else
   Menu(mb);

 modbus_close(mb);
 modbus_free(mb);
 return 0;
}
