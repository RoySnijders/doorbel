/*
 * File:   mp3.c
 * Author: Roy Snijders
 * Version  1.0
 
 *
 * Created on 1 December 2024
 */

#ifndef MP3_H
#define	MP3_H

/* Function Prototypes */
#define MP3_STATUS_INIT_ERROR	0
#define MP3_STATUS_READY		1


#define DFPLAYER_EQ_NORMAL 0
#define DFPLAYER_EQ_POP 1
#define DFPLAYER_EQ_ROCK 2
#define DFPLAYER_EQ_JAZZ 3
#define DFPLAYER_EQ_CLASSIC 4
#define DFPLAYER_EQ_BASS 5

#define DFPLAYER_DEVICE_U_DISK 1
#define DFPLAYER_DEVICE_SD 2
#define DFPLAYER_DEVICE_AUX 3
#define DFPLAYER_DEVICE_SLEEP 4
#define DFPLAYER_DEVICE_FLASH 5

//#define _DEBUG
#define DFPlayerTimeOut 0
#define DFPlayerWrongStack 1
#define DFPlayerCardInserted 2
#define DFPlayerCardRemoved 3
#define DFPlayerCardOnline 4
#define DFPlayerPlayFinished 5
#define DFPlayerError 6
#define DFPlayerUSBInserted 7
#define DFPlayerUSBRemoved 8
#define DFPlayerUSBOnline 9
#define DFPlayerCardUSBOnline 10
#define DFPlayerFeedBack 11

#define DFPlayerBusy 1
#define DFPlayerSleeping 2
#define DFPlayerSerialWrongStack 3
#define DFPlayerCheckSumNotMatch 4
#define DFPlayerFileIndexOut 5
#define DFPlayerFileMismatch 6
#define DFPlayerAdvertise 7

#define Stack_Header 0
#define Stack_Version 1
#define Stack_Length 2
#define Stack_Command 3
#define Stack_ACK 4
#define Stack_Parameter 5
#define Stack_CheckSum 7
#define Stack_End 9

#define MP3_RX_LEN 	10
#define MP3_TX_LEN	10

uint8_t mp3_init(void);
uint8_t mp3_read(uint8_t *command,uint16_t *data,uint16_t timeout);
uint8_t mp3_send(uint8_t command,uint8_t ack,uint16_t data);
uint16_t  mp3_checksum(uint8_t *buf);
uint8_t mp3_checkready(uint16_t timeout);
uint8_t mp3_play(uint8_t val);
uint8_t mp3_volume(uint8_t val);
uint8_t  mp3_stop(void);
uint8_t  mp3_pause(void);
uint8_t  mp3_sleep(void);
uint8_t  mp3_reset(void);




#endif	/* MP3_H */

