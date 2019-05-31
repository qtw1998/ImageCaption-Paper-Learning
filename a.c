/** ###################################################################
**     Filename  : LININGa.c
**     Project   : LININGa
**     Processor : MC9S12XS128MAL
**     Version   : Driver 01.14
**     Compiler  : CodeWarrior HCS12X C Compiler
**     Date/Time : 2016/3/21, 22:01
**     Abstract  :
**         Main module.
**         This module contains user's application code.
**     Settings  :
**     Contents  :
**         No public methods
**
** ###################################################################*/
/* MODULE LININGa */

/* Including needed modules to compile this module/procedure */
#include <stdlib.h>
#include "Cpu.h"
#include "Events.h"
#include "AS1.h"
#include "BT232.h"
#include "QESA.h"
#include "QESB.h"
#include "QESP.h"
#include "LED2.h"
#include "DSP1.h"
#include "DSP2.h"
#include "SVR1.h"
#include "SVR2.h"
#include "BEEP.h"
#include "AD1.h"
#include "DRVC1.h"
#include "DRVC2.h"
#include "DRVS.h"
#include "DRVE.h"
#include "IRG1.h"
#include "IRG2.h"
#include "IRG3.h"
#include "DSW.h"
#include "TI1.h"
#include "PG.h"
#include "S2.h"
#include "MBTimer.h"
/* Include shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"

/* User includes (#include below this line is not maintained by Processor Expert) */
static byte DSPTable[] = { 0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90, 0x88, 0x83, 0xA7, 0xA1, 0x86, 0x8E };

int SlaveAddr=1;
unsigned char ModBusCommand[100];
unsigned char ModBusMessage[100];
int ModBusIdx=-1;
unsigned char ModTimeout=25;
unsigned char ModAddr=1;    // HMI device ID
unsigned char ModFunction;
unsigned char ModData[80];
int ModDataLen=0;
word RcvByte, SndByte;

unsigned char StopRun=0, CtrlMode=0;    // 0: Stopped; 1: Running || CtrlMode 0: Speed/Angle; CtrlMode 1: Odometry pulses
unsigned int TimeWindow=0;  // communication monitor
unsigned char ErrCode=0;    // ErrCode TBD
unsigned int VBat;        // Battery Voltage
unsigned int RealSpeed;   // Encoder feedback
unsigned int RealAngle;
unsigned int deltaPulse=0, deltaT=0;
unsigned long long PG_Accumulated=0;
unsigned long long PG_Target=0;
unsigned long long PG_Togo=0;

unsigned char CmdRun=0;
unsigned char CmdSpeed=0;
unsigned char TargetSpeed=0;
unsigned char TargetAngle=0;
unsigned char VDir=0, SDir=0; // VDir 0:forward; 1:backward || SDir 0:Left; 1:Right
unsigned char CtrlVar=0;
unsigned char AIN[3];

// ÒÔ16½øÖÆ·½Ê½ÔÚÁ½Î»ÊýÂë¹ÜÉÏÏÔÊ¾ÊýÖµ,ÊýÖµ·¶Î§0x00~0xFF(¶ÔÓ¦Ê®½øÖÆÊýÖµ·¶Î§0~255)
void ShowNumHEX(byte num) {
  unsigned char h, l;
  h = DSPTable[num>>4];
  l = DSPTable[num&0x0F];
  DSP1_PutVal(h);
  DSP2_PutVal(l);
}

// ÒÔÊ®½øÖÆ·½Ê½ÔÚÁ½Î»ÊýÂë¹ÜÉÏÏÔÊ¾ÊýÖµ,ÊýÖµ·¶Î§0x00~0x99
void ShowNumDEC(byte num) {
  unsigned char h, l;
  h = DSPTable[(num / 10) % 10];
  l = DSPTable[num % 10];
  DSP1_PutVal(h);
  DSP2_PutVal(l);
}

// ¶ÔÓ¦F3µÄ¹¦ÄÜ·ÖÖ§,µç»úËÙ¶È±Õ»·¿ØÖÆ²âÊÔ
// Switch Motor drive closed-loop test
// ÓÉÊýÂë¿ª¹Ø¿ØÖÆµç»úËÙ¶È,¿ØÖÆ±äÁ¿CtrlVar:ÊýÂë¿ª¹Ø¿ØÖÆ,Ë³Ê±ÕëÔö¼Ó,ÄæÊ±Õë¼õÐ¡
// CtrlVar: CW to increase CCW to decrease
void Test_MotorClosedLoop() {
  unsigned int TestSpeed;
  
  TestSpeed = RealSpeed/2;    // µç»úµ±Ç°ÕæÊµÔË×ªËÙ¶È RealSpeed
  
  
  if (CtrlVar > 99) CtrlVar = 99;     // ËÙ¶ÈÖ¸ÁîÏÞ·ù SpeedLimit
  if (!QESP_GetVal()) CtrlVar = 0;
  //DRVC1_SetRatio8(CtrlVar);
//  if (CtrlVar>=10) {
  // ËÙ¶È±Õ»·¿ØÖÆ²Ù×÷Âß¼­
  if (CtrlVar>TestSpeed) CmdSpeed++;
  else if (CmdSpeed>0) CmdSpeed--;
  //DRVC1_SetRatio8(CmdSpeed);
  //DRVC1_SetRatio16(CmdSpeed);
  DRVC2_SetRatio16(CmdSpeed);
}

/* CRC16 Table High byte */
static unsigned char CRC16Hi[] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
} ;

/* CRC16 Table Low byte */
static char CRC16Lo[] = {
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2,
0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6,
0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE,
0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA,
0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62,
0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE,
0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76,
0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A,
0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86,
0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
} ;

unsigned char CRCHi ; /* high byte of CRC initialized */
unsigned char CRCLo ; /* low byte of CRC initialized */
unsigned int GetCRC16(unsigned char *puchMsg, int DataLen)
{
  unsigned Index ; /* will index into CRC16 lookup table */

  CRCHi = 0xFF ; /* high byte of CRC16 initialized */
  CRCLo = 0xFF ; /* low byte of CRC16 initialized */
  while (DataLen--){
    Index = CRCHi ^ *puchMsg++ ; /* calculate the CRC16 */
    CRCHi = CRCLo ^ CRC16Hi[Index] ;
    CRCLo = CRC16Lo[Index] ;
  }
  return ((unsigned)CRCHi << 8 | CRCLo) ;
}

// SendRtuCmdToModBus will add CRC16
unsigned char SendRtuCmdToModBus(unsigned char *cmd, int len)
{ 
   unsigned CRC16;
   word Snd;
   CRC16 = GetCRC16(cmd,len);
   AS1_ClearTxBuf();
   AS1_SendBlock(cmd,len,&Snd);
   AS1_SendChar(CRCHi);  /* send CRC16 high */
   AS1_SendChar(CRCLo);  /* send CRC16 low */
   return 1;
}

int ModBusMessageClr()
{
  unsigned char k;
  for (k=0;k<100;k++) 
    ModBusMessage[k] = 0;
  return 1;  
}

/* Function code 02: ¶ÁÐ¡³µÔËÐÐ/Í£»ú¡¢µ±Ç°ÔËÐÐÄ£Ê½µÈ×´Ì¬   : 1x */
/* Function code 04: ¶ÁÐ¡³µÊµ¼ÊËÙ¶È¡¢Êµ¼Ê¶æ½Ç¡¢Ê£ÓàÀï³Ì¡¢µç³ØµçÑ¹µÈ×´Ì¬  : 3x */
/* Function code 15: Éè¶¨Ð¡³µÆôÍ£¡¢Ä£Ê½¡¢Àï³ÌÇåÁãµÈÖ¸Áî : 0x */
/* Function code 16: Éè¶¨Ð¡³µÄ¿±êËÙ¶È¡¢¶æ½Ç¡¢Àï³ÌµÈÐÅÏ¢ : 4x */
unsigned char ResponseModCmd(void)
{
  unsigned int StartAddr, PointNo;
  unsigned int DataByteCount;
  unsigned char CtrlCmd, CtrlCmd2;
  unsigned long long tempH, tempL;

  if(ModAddr == SlaveAddr) {  // Response on address matching
    switch(ModFunction) {
      case 2:  // DI: ¶ÁÐ¡³µÔËÐÐ/Í£»ú¡¢µ±Ç°ÔËÐÐÄ£Ê½¡¢¹ÊÕÏ×´Ì¬µÈ×´Ì¬
        StartAddr = (ModData[0]<<8) + ModData[1];
        PointNo = (ModData[2]<<8) + ModData[3];
        if((StartAddr==0x10)&&(PointNo == 0x04)) {    // Limited condition
          DataByteCount = 1;          // 1 byte
          ModBusCommand[0] = ModAddr;
          ModBusCommand[1] = ModFunction;
          ModBusCommand[2] = DataByteCount;
          ModBusCommand[3] = (ErrCode<<2) + (CtrlMode<<1) + CmdRun;
          SendRtuCmdToModBus(ModBusCommand, 4);
        }
        break;
      case 4:  /* AI: ¶ÁÐ¡³µÊµ¼ÊËÙ¶È¡¢Êµ¼Ê¶æ½Ç¡¢Ê£ÓàÀï³Ì¡¢µç³ØµçÑ¹µÈ×´Ì¬ */
        StartAddr = (ModData[0]<<8) + ModData[1];
        PointNo = (ModData[2]<<8) + ModData[3];
        if((StartAddr==0x05)&&(PointNo == 0x05)) {    // Limited condition
          DataByteCount = PointNo * 2;     /* 2 bytes for one AI channel */
          ModBusCommand[0] = ModAddr;
          ModBusCommand[1] = ModFunction;
          ModBusCommand[2] = DataByteCount;
          ModBusCommand[3] = VDir;    //
          ModBusCommand[4] = RealSpeed&0xFF;
          ModBusCommand[5] = SDir;    //
          ModBusCommand[6] = RealAngle&0xFF;
          ModBusCommand[7] = (PG_Togo>>24)&0xFF;
          ModBusCommand[8] = (PG_Togo>>16)&0xFF;
          ModBusCommand[9] = (PG_Togo>>8)&0xFF;
          ModBusCommand[10] = PG_Togo&0xFF;
          ModBusCommand[11] = VBat>>8&0xFF;
          ModBusCommand[12] = VBat&0xFF;
          SendRtuCmdToModBus(ModBusCommand, 13);
        }
        break;
      case 15:  /* Éè¶¨Ð¡³µÆôÍ£¡¢Ä£Ê½¡¢Àï³ÌÇåÁãµÈÖ¸Áî */
        StartAddr = (ModData[0]<<8) + ModData[1];
        PointNo = (ModData[2]<<8) + ModData[3];
        if((StartAddr==0x10)&&(PointNo == 0x04)) {    // Limited condition
          CtrlCmd = ModData[5];
          if (CtrlCmd & 0x01) CmdRun = 1; else CmdRun = 0;
          if (CtrlCmd & 0x02) CtrlMode = 1; else CtrlMode = 0;
          if (CtrlCmd & 0x04) { PG_Togo = 0; PG_Target = 0; PG_Accumulated = 0; }
          if (CtrlCmd & 0x08) ErrCode = 0;
          ModBusCommand[0] = ModAddr;
          ModBusCommand[1] = ModFunction;
          ModBusCommand[2] = ModData[0];
          ModBusCommand[3] = ModData[1];
          ModBusCommand[4] = ModData[2];
          ModBusCommand[5] = ModData[3];
          ModBusCommand[6] = ModData[4];
          SendRtuCmdToModBus(ModBusCommand, 7);
        }
        break;
      case 16:  /* Éè¶¨Ð¡³µÄ¿±êËÙ¶È¡¢¶æ½Ç¡¢Àï³ÌµÈÐÅÏ¢ */
        StartAddr = (ModData[0]<<8) + ModData[1];
        PointNo = (ModData[2]<<8) + ModData[3];
        DataByteCount = ModData[4];
        if((StartAddr==0x01)&&(PointNo == 0x02)) {    // Limited condition
          DataByteCount = ModData[4];
          if (ModData[5]) VDir = 1;  else VDir = 0;
          TargetSpeed = ModData[6];
          if (ModData[7]) SDir = 1;  else SDir = 0;
          TargetAngle = ModData[8];
          ModBusCommand[0] = ModAddr;
          ModBusCommand[1] = ModFunction;
          ModBusCommand[2] = ModData[0];
          ModBusCommand[3] = ModData[1];
          ModBusCommand[4] = ModData[2];
          ModBusCommand[5] = ModData[3];
          SendRtuCmdToModBus(ModBusCommand, 6);
        }
        if ((StartAddr==0x03)&&(PointNo == 0x02)) {
          DataByteCount = ModData[4];
          tempH = (ModData[5]<<8) + ModData[6];
          tempL = (ModData[7]<<8) + ModData[8];
          PG_Target = (tempH<<16) + tempL;
          PG_Accumulated = 0;
          ModBusCommand[0] = ModAddr;
          ModBusCommand[1] = ModFunction;
          ModBusCommand[2] = ModData[0];
          ModBusCommand[3] = ModData[1];
          ModBusCommand[4] = ModData[2];
          ModBusCommand[5] = ModData[3];
          SendRtuCmdToModBus(ModBusCommand, 6);

          AS1_SendChar(0xAA);
          AS1_SendChar((PG_Accumulated>>24)&0xFF);
          AS1_SendChar((PG_Accumulated>>16)&0xFF);
          AS1_SendChar((PG_Accumulated>>8)&0xFF);
          AS1_SendChar(PG_Accumulated&&0xFF);
          AS1_SendChar(0xBB);
          AS1_SendChar((PG_Target>>24)&0xFF);
          AS1_SendChar((PG_Target>>16)&0xFF);
          AS1_SendChar((PG_Target>>8)&0xFF);
          AS1_SendChar(PG_Target&0xFF);
          AS1_SendChar(0xCC);

        }
        break;
      case 17:  /* Report Slave ID */
        ModBusCommand[0] = SlaveAddr;
        ModBusCommand[1] = ModFunction;
        ModBusCommand[2] = 3;
        ModBusCommand[3] = 18;    // Slave ID
        ModBusCommand[4] = 255;
        ModBusCommand[5] = 255;
        SendRtuCmdToModBus(ModBusCommand, 6);
        ShowNumHEX(0xF5);
        break;
      default:
        break;
      /* Add the function you need to response */
      //
    }
  }
  return 1;
}

unsigned char CheckModBusRtu(void)
{
  word t;
  unsigned char k;

  while (AS1_GetCharsInRxBuf()!=0) {
    AS1_RecvChar(&ModBusMessage[++ModBusIdx]);
    MBTimer_Enable();
    MBTimer_Reset();
  }
  if (ModBusIdx > 0) {
    MBTimer_GetTimeMS(&t);
    if (t > ModTimeout) {           // Check for a complete Modbus query
      GetCRC16(ModBusMessage, ModBusIdx-1);
      if((CRCHi==ModBusMessage[ModBusIdx-1]) && (CRCLo==ModBusMessage[ModBusIdx])) {  // Valid ModBus CMD
        TimeWindow = 0;   // clear communication WDT
        ModAddr = ModBusMessage[0];
        ModFunction = ModBusMessage[1];
        for(k=2;k<=ModBusIdx-2;k++){
          ModData[k-2]=ModBusMessage[k];
        }
        ModDataLen = ModBusIdx-3;
        ResponseModCmd();           // ModBus CMD response
      } else {    // CRC16 error
        ModDataLen = 0;
      }
      ModBusIdx=-1;
      MBTimer_Disable();
    }
  }
  return 1;
}

// Ö÷³ÌÐòÈë¿Ú
void main(void)
{
  /* Write your local variable definition here */
  unsigned int i=0;
  unsigned char CmdCode=0, CmdValid=0;
  unsigned char tempChar, tempH, tempL;

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/

  /* Write your code here */
  tempChar=0;
  
  DRVE_PutVal(0);       // Disable motor driving µç»úÊ¹ÄÜ 0-Disable / 1 - Enable
  DRVC2_SetRatio8(0);   // Run forward µç»úÕý×ªËÙ¶È 0-stop 255-FullSpeed
  DRVC1_SetRatio8(0);   // Run backward µç»ú·´×ªËÙ¶È 0-stop 255-FullSpeed
  
  BEEP_PutVal(1);       // Beeper ON ·äÃùÆ÷Ïì
  Cpu_Delay100US(2000); // ÏµÍ³ÑÓÊ± 200ms
  BEEP_PutVal(0);       // Beeper OFF ·äÃùÆ÷¹Ø±Õ
  
  PG_Accumulated = 0;
  deltaT = 0;
  TimeWindow = 0;
  ErrCode = 0;

  DRVE_PutVal(1);       // Disable motor driving ½ûÖ¹µç»úÇý¶¯

  // ³ÌÐòÖ÷Ñ­»· MCU task Main Loop
  while (1) {
    CheckModBusRtu();
    Cpu_Delay100US(5);
/*
    if (deltaT>=50) {
      AS1_SendChar(00);
      AS1_SendChar((i>>8)&0xFF);
      AS1_SendChar(i&0xFF);
      deltaT = 0;
    }
*/
    ShowNumHEX((CtrlMode<<4)+CmdRun);
    
    if (TimeWindow>500) CmdRun=0;   // stop if no valid command for 10s (0.02ms*500)

    AD1_Measure(1);
    AD1_GetValue8(&AIN);
    VBat = AIN[0];
  }

  DRVE_PutVal(0);       // Disable motor driving ½ûÖ¹µç»úÇý¶¯
  DRVC2_SetRatio8(0);   // Run forward 0  ËÙ¶ÈÎª0
  DRVC1_SetRatio8(0);   // Run backward 0 ËÙ¶ÈÎª0
  SVR2_SetDutyUS(1500); // Steer midpos ¶æ»ú¶æ½Ç»ØÖÐÎ»

  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;){}
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END LININGa */
/*
** ###################################################################
**
**     This file was created by Processor Expert 3.02 [04.44]
**     for the Freescale HCS12X series of microcontrollers.
**
** ###################################################################
*/
