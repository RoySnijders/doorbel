/*
* File:   mp3.c
* Author: Roy Snijders
* Version  1.0
*
* Created on December 27, 2024
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "main.h"
#include "mp3.h"

extern UART_HandleTypeDef huart1;
#define mp3 &huart1


uint8_t mp3_rxbuf[MP3_RX_LEN];
uint8_t mp3_txbuf[MP3_TX_LEN] = {0x7E, 0xFF, 06, 00, 01, 00, 00, 00, 00, 0xEF};
uint8_t	mp3_status;
uint8_t mp3_command;
uint16_t mp3_data;


//----------------------------------------------------------
uint8_t mp3_init(void)
{	//Initializing DFPlayer ... (May take 3~5 seconds)

	if(mp3_checkready(2000)!=0)
		return 1;	//@TODO
	// the module is online
	return 0;
}




//----------------------------------------------------------------------
//return 0 = OK
//----------------------------------------------------------------------
uint8_t mp3_read(uint8_t *command,uint16_t *data,uint16_t timeout)
{	//0x7E, 0xFF, 06, 00, 01, 00, 00, 00, 00, 0xEF
	if(HAL_UART_Receive(mp3, mp3_rxbuf, MP3_RX_LEN, timeout)==HAL_OK) 		//if transfer is successful
	{	if(mp3_rxbuf[0]!=0x7E) return 2;							// error code 2 - wrong start byte
		if(mp3_rxbuf[1]!=0xFF) return 3;							// error code 3 - wrong version
		if(mp3_rxbuf[2]!=0x06) return 4;							// error code 4 - wrong length
		*command=mp3_rxbuf[3];
		//feedback byte = [4]
		//data=[5+6];												// 2 bytes of data
		uint16_t checksum= mp3_checksum(mp3_rxbuf);
		if(mp3_rxbuf[7]!=(uint8_t) (checksum>>8))	return 5;			//error in MSB checksum
		if(mp3_rxbuf[8]!=(uint8_t) checksum)			return 6;			//error in LSB checksum
		if(mp3_rxbuf[9]!=0xEF) return 7;							// error code 4 - wrong stop byte
		//process data
		*data=(uint16_t) (mp3_rxbuf[5]*256) + mp3_rxbuf[6]; 	// read directly from array
	} else
		return 8;													// error time out
	return 0;														// normal exist  = 0
}

//----------------------------------------------------------------------
//return 0 = OK
//----------------------------------------------------------------------
uint8_t mp3_send(uint8_t command,uint8_t ack,uint16_t data)
{	//0x7E, 0xFF, 06, 00, 01, 00, 00, 00, 00, 0xEF

	mp3_txbuf[3]=command;
	mp3_txbuf[4]=ack;
	mp3_txbuf[5]=(uint8_t) (data>>8);
	mp3_txbuf[6]=(uint8_t) data;
	uint16_t checksum = mp3_checksum(mp3_txbuf);
	mp3_txbuf[7]=(uint8_t) (checksum>>8);
	mp3_txbuf[8]=(uint8_t) checksum;

	if(HAL_UART_Transmit(mp3, mp3_txbuf, MP3_TX_LEN, 100)==HAL_OK) 		//if transfer is successful
		return 0;															// normal exit  = 0
	else
		return 1;															// error = 1
}

//----------------------------------------------------------------------
//Checksum (2 bytes) = 0xFFFF–(Ver.+Length+CMD+Feedback+Para_MSB+Para_LSB)+1
//----------------------------------------------------------------------
uint16_t  mp3_checksum(uint8_t *buf)
{ 	return (0xFFFF-(buf[1]+buf[2]+buf[3]+buf[4]+buf[5]+buf[6])+1);
}


//----------------------------------------------------------------------
//When the module is working, users can use the command as above (0x3F) to query the status of the online storage
//devices. For example, if the module returns the data 7E FF 06 3F 00 00 0A xx xx EF, LSB 0x0A(0000 1010)
//represents SD card online. If LSB is 0x1F(0000 1111), it represents all of USB flash drive, SD card, and PC
//online(PC online means module is connecting with PC via a USB cable).
//----------------------------------------------------------------------
uint8_t  mp3_checkready(uint16_t timeout)
{	mp3_command=0x3F;
	mp3_data=0;

	mp3_send(0x3F, 0x00, 0);
	if(mp3_read(&mp3_command,&mp3_data,2000)!=0) return 2;	//example timeout
	if(((uint8_t) mp3_data)== 0x02)							//SD card is online
	{	mp3_status=MP3_STATUS_READY;
		return 0;
	}
	mp3_status=MP3_STATUS_INIT_ERROR;	//Unable to begin: recheck the connection/insert the SD card!
	return 1;
}


//----------------------------------------------------------------------
uint8_t  mp3_play(uint8_t val)
{	return mp3_send(0x03,0,val);

}

uint8_t  mp3_volume(uint8_t val)
{	return mp3_send(0x06,0,val);
}

uint8_t  mp3_stop(void)
{	return mp3_send(0x16,0,0);
}

uint8_t  mp3_pause(void)
{	return mp3_send(0x0E,0,0);
}

uint8_t  mp3_sleep(void)
{	return mp3_send(0x0A,0,0);
}

uint8_t  mp3_reset(void)
{	return mp3_send(0x0C,0,0);
}

//-----------------------------------------------------------------------
//void printDetail(uint8_t type, int value)
//{
//  switch (type)
//  {	case TimeOut:	      Serial.println(F("Time Out!"));      break;
//    case WrongStack:      Serial.println(F("Stack Wrong!"));      break;
//    case DFPlayerCardInserted:      Serial.println(F("Card Inserted!"));      break;
//    case DFPlayerCardRemoved:      Serial.println(F("Card Removed!"));      break;
//    case DFPlayerCardOnline:      Serial.println(F("Card Online!"));      break;
//    case DFPlayerUSBInserted:      Serial.println("USB Inserted!");      break;
//    case DFPlayerUSBRemoved:      Serial.println("USB Removed!");      break;
//    case DFPlayerPlayFinished:      Serial.print(F("Number:")); Serial.print(value);Serial.println(F(" Play Finished!"));      break;
//    case DFPlayerError:Serial.print(F("DFPlayerError:"));
//      switch (value)
//      {	case Busy:				Serial.println(F("Card not found"));     break;
//        case Sleeping:  		Serial.println(F("Sleeping"));          break;
//        case SerialWrongStack:  Serial.println(F("Get Wrong Stack"));       break;
//        case CheckSumNotMatch:	Serial.println(F("Check Sum Not Match"));          break;
//        case FileIndexOut:		Serial.println(F("File Index Out of Bound"));          break;
//        case FileMismatch:		Serial.println(F("Cannot Find File"));          break;
//        case Advertise:			Serial.println(F("In Advertise"));         break;
//        default:         break;
//      }
//  }
//}
